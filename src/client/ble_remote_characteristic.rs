use core::borrow::Borrow;

use super::ble_client::BLEClientState;
use super::ble_remote_service::BLERemoteServiceState;
use super::{BLEReader, BLEWriter};
use crate::BLEAttribute;
use crate::{
  ble,
  utilities::{as_void_ptr, voidp_to_ref, ArcUnsafeCell, BleUuid, WeakUnsafeCell},
  BLEError, BLERemoteDescriptor, Signal,
};
use alloc::{boxed::Box, vec::Vec};
use bitflags::bitflags;
use core::ffi::c_void;
use esp_idf_svc::sys as esp_idf_sys;

bitflags! {
  #[repr(transparent)]
  #[derive(Debug, Clone, Copy, PartialEq, Eq)]
  pub struct GattCharacteristicProperties: u8 {
    const BROADCAST = esp_idf_sys::BLE_GATT_CHR_PROP_BROADCAST as _;
    const READ = esp_idf_sys::BLE_GATT_CHR_PROP_READ as _;
    const WRITE_NO_RSP = esp_idf_sys::BLE_GATT_CHR_PROP_WRITE_NO_RSP as _;
    const WRITE = esp_idf_sys::BLE_GATT_CHR_PROP_WRITE as _;
    const NOTIFY = esp_idf_sys::BLE_GATT_CHR_PROP_NOTIFY as _;
    const INDICATE = esp_idf_sys::BLE_GATT_CHR_PROP_INDICATE as _;
    const AUTH_SIGN_WRITE = esp_idf_sys::BLE_GATT_CHR_PROP_AUTH_SIGN_WRITE as _;
    const EXTENDED =  esp_idf_sys::BLE_GATT_CHR_PROP_EXTENDED as _;
  }
}

#[allow(clippy::type_complexity)]
pub struct BLERemoteCharacteristicState {
  pub(crate) service: WeakUnsafeCell<BLERemoteServiceState>,
  pub(crate) uuid: BleUuid,
  pub handle: u16,
  end_handle: u16,
  properties: GattCharacteristicProperties,
  descriptors: Option<Vec<BLERemoteDescriptor>>,
  signal: Signal<u32>,
  on_notify: Option<Box<dyn FnMut(&[u8]) + Send + Sync>>,
}

impl BLEAttribute for BLERemoteCharacteristicState {
  fn get_client(&self) -> Option<ArcUnsafeCell<BLEClientState>> {
    match self.service.upgrade() {
      Some(x) => x.get_client(),
      None => None,
    }
  }
}

impl BLERemoteCharacteristicState {
  pub fn conn_handle(&self) -> u16 {
    match self.service.upgrade() {
      Some(x) => x.conn_handle(),
      None => esp_idf_sys::BLE_HS_CONN_HANDLE_NONE as _,
    }
  }
}

#[derive(Clone)]
pub struct BLERemoteCharacteristic {
  state: ArcUnsafeCell<BLERemoteCharacteristicState>,
}

impl BLERemoteCharacteristic {
  pub(crate) fn new(
    service: WeakUnsafeCell<BLERemoteServiceState>,
    chr: &esp_idf_sys::ble_gatt_chr,
  ) -> Self {
    Self {
      state: ArcUnsafeCell::new(BLERemoteCharacteristicState {
        service,
        uuid: BleUuid::from(chr.uuid),
        handle: chr.val_handle,
        end_handle: 0,
        properties: GattCharacteristicProperties::from_bits_truncate(chr.properties),
        descriptors: None,
        signal: Signal::new(),
        on_notify: None,
      }),
    }
  }

  pub(crate) fn state(&self) -> &BLERemoteCharacteristicState {
    &self.state
  }

  pub fn uuid(&self) -> BleUuid {
    self.state.uuid
  }

  pub fn properties(&self) -> GattCharacteristicProperties {
    self.state.properties
  }

  pub async fn get_descriptors(
    &mut self,
  ) -> Result<core::slice::IterMut<'_, BLERemoteDescriptor>, BLEError> {
    if self.state.descriptors.is_none() {
      self.state.descriptors = Some(Vec::new());

      if self.state.end_handle == 0 {
        unsafe {
          ble!(esp_idf_sys::ble_gattc_disc_all_chrs(
            self.state.conn_handle(),
            self.state.handle,
            self.state.service.upgrade().unwrap().end_handle,
            Some(Self::next_char_cb),
            as_void_ptr(self),
          ))?;
        }

        ble!(self.state.signal.wait().await)?;
      }

      if self.state.handle != self.state.end_handle {
        unsafe {
          ble!(esp_idf_sys::ble_gattc_disc_all_dscs(
            self.state.conn_handle(),
            self.state.handle,
            self.state.end_handle,
            Some(Self::descriptor_disc_cb),
            as_void_ptr(self),
          ))?;
        }
        ble!(self.state.signal.wait().await)?;
      }
    }

    Ok(self.state.descriptors.as_mut().unwrap().iter_mut())
  }

  pub async fn get_descriptor(
    &mut self,
    uuid: BleUuid,
  ) -> Result<&mut BLERemoteDescriptor, BLEError> {
    let mut iter = self.get_descriptors().await?;
    iter
      .find(|x| x.uuid() == uuid)
      .ok_or_else(|| BLEError::fail().unwrap_err())
  }

  extern "C" fn next_char_cb(
    conn_handle: u16,
    error: *const esp_idf_sys::ble_gatt_error,
    chr: *const esp_idf_sys::ble_gatt_chr,
    arg: *mut c_void,
  ) -> i32 {
    let characteristic = unsafe { voidp_to_ref::<Self>(arg) };
    if characteristic.state.conn_handle() != conn_handle {
      return 0;
    }

    let error = unsafe { &*error };
    if error.status == 0 {
      characteristic.state.end_handle = unsafe { (*chr).def_handle - 1 };
    } else if error.status == esp_idf_sys::BLE_HS_EDONE as _ {
      characteristic.state.end_handle = characteristic.state.service.upgrade().unwrap().end_handle;
    }

    characteristic.state.signal.signal(error.status as _);
    esp_idf_sys::BLE_HS_EDONE as _
  }

  extern "C" fn descriptor_disc_cb(
    conn_handle: u16,
    error: *const esp_idf_sys::ble_gatt_error,
    _chr_val_handle: u16,
    dsc: *const esp_idf_sys::ble_gatt_dsc,
    arg: *mut c_void,
  ) -> i32 {
    let characteristic = unsafe { voidp_to_ref::<Self>(arg) };
    if characteristic.state.conn_handle() != conn_handle {
      return 0;
    }

    let error = unsafe { &*error };
    let dsc = unsafe { &*dsc };

    if error.status == 0 {
      let descriptor =
        BLERemoteDescriptor::new(ArcUnsafeCell::downgrade(&characteristic.state), dsc);
      characteristic
        .state
        .descriptors
        .as_mut()
        .unwrap()
        .push(descriptor);
      return 0;
    }

    characteristic.state.signal.signal(error.status as _);
    esp_idf_sys::BLE_HS_EDONE as _
  }

  pub async fn read_value(&mut self) -> Result<Vec<u8>, BLEError> {
    let mut reader = BLEReader::new(self.state.conn_handle(), self.state.handle);
    reader.read_value().await
  }

  pub async fn write_value(&mut self, data: &[u8], response: bool) -> Result<(), BLEError> {
    let mut writer = BLEWriter::new(self.state.conn_handle(), self.state.handle);
    writer.write_value(data, response).await
  }

  pub async fn subscribe_notify(&mut self, response: bool) -> Result<(), BLEError> {
    self.set_notify(0x01, response).await
  }

  pub async fn subscribe_indicate(&mut self, response: bool) -> Result<(), BLEError> {
    self.set_notify(0x02, response).await
  }

  pub async fn unsubscribe(&mut self, response: bool) -> Result<(), BLEError> {
    self.set_notify(0x00, response).await
  }

  async fn set_notify(&mut self, val: u16, response: bool) -> Result<(), BLEError> {
    let desc = self.get_descriptor(BleUuid::from_uuid16(0x2902)).await?;
    desc.write_value(&val.to_ne_bytes(), response).await
  }

  pub fn on_notify(&mut self, callback: impl FnMut(&[u8]) + Send + Sync + 'static) -> &mut Self {
    self.state.on_notify = Some(Box::new(callback));
    self
  }

  pub fn can_notify(&self) -> bool {
    self
      .properties()
      .contains(GattCharacteristicProperties::NOTIFY)
  }

  pub fn can_indicate(&self) -> bool {
    self
      .properties()
      .contains(GattCharacteristicProperties::INDICATE)
  }

  pub fn can_read(&self) -> bool {
    self
      .properties()
      .contains(GattCharacteristicProperties::READ)
  }

  pub fn can_write(&self) -> bool {
    self
      .properties()
      .contains(GattCharacteristicProperties::WRITE)
  }

  pub fn can_write_no_response(&self) -> bool {
    self
      .properties()
      .contains(GattCharacteristicProperties::WRITE_NO_RSP)
  }

  pub fn can_broadcast(&self) -> bool {
    self
      .properties()
      .contains(GattCharacteristicProperties::BROADCAST)
  }

  pub(crate) unsafe fn notify(&mut self, mut om: *mut esp_idf_sys::os_mbuf) {
    if let Some(no_notify) = self.state.on_notify.as_mut() {
      let mut buf = Vec::with_capacity(esp_idf_sys::BLE_ATT_ATTR_MAX_LEN as _);

      while !om.is_null() {
        let slice = unsafe { core::slice::from_raw_parts((*om).om_data, (*om).om_len as _) };
        buf.extend_from_slice(slice);
        om = unsafe { (*om).om_next.sle_next };
      }

      no_notify(&buf);
    }
  }
}

impl core::fmt::Display for BLERemoteCharacteristic {
  fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
    write!(f, "BLERemoteCharacteristic[{}]", self.state.uuid)
  }
}

impl core::fmt::Debug for BLERemoteCharacteristic {
  fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
    f.debug_struct("BLERemoteCharacteristic")
      .field("uuid", &self.state.uuid)
      .field(
        "service",
        &self.state.service.upgrade().map(|svc| svc.borrow().uuid),
      )
      .field("handle", &self.state.handle)
      .field("end_handle", &self.state.end_handle)
      .field("properties", &self.state.properties)
      .finish()
  }
}
