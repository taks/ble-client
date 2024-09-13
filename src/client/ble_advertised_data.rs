use alloc::vec::Vec;
use bstr::BStr;
use esp_idf_svc::sys;

use crate::{enums::AdvFlag, utilities::BleUuid};

pub struct BLEAdvertisedData<T>
where
  T: AsRef<[u8]>,
{
  payload: T,
}

impl<T: AsRef<[u8]>> BLEAdvertisedData<T> {
  pub(crate) fn new(payload: T) -> Self {
    Self { payload }
  }

  pub fn payload(&self) -> &[u8] {
    self.payload.as_ref()
  }

  #[allow(clippy::should_implement_trait)]
  pub fn clone(&self) -> BLEAdvertisedData<Vec<u8>> {
    BLEAdvertisedData::<Vec<u8>>::new(self.payload().to_vec())
  }

  /// Get the advertisement flags.
  pub fn adv_flags(&self) -> Option<AdvFlag> {
    let data = self
      .decode()
      .find(|x| x.ty == (sys::BLE_HS_ADV_TYPE_FLAGS as _))?;

    let ad_flag = data.data.first()?;
    AdvFlag::from_bits(*ad_flag)
  }

  #[allow(unused_variables)]
  pub fn is_advertising_service(&self, uuid: &BleUuid) -> bool {
    todo!()
  }

  pub fn service_uuids(&self) -> Vec<BleUuid> {
    let mut ret = Vec::new();
    self.decode().for_each(|x| match x.ty as u32 {
      sys::BLE_HS_ADV_TYPE_INCOMP_UUIDS16 | sys::BLE_HS_ADV_TYPE_COMP_UUIDS16 => {
        let mut data = x.data;
        while data.len() >= 2 {
          let (uuid, data_) = data.split_at(2);
          ret.push(BleUuid::from_uuid16(u16::from_le_bytes(
            uuid.try_into().unwrap(),
          )));
          data = data_;
        }
      }
      sys::BLE_HS_ADV_TYPE_INCOMP_UUIDS32 | sys::BLE_HS_ADV_TYPE_COMP_UUIDS32 => {
        let mut data = x.data;
        while data.len() >= 4 {
          let (uuid, data_) = data.split_at(4);
          ret.push(BleUuid::from_uuid32(u32::from_le_bytes(
            uuid.try_into().unwrap(),
          )));
          data = data_;
        }
      }
      sys::BLE_HS_ADV_TYPE_INCOMP_UUIDS128 | sys::BLE_HS_ADV_TYPE_COMP_UUIDS128 => {
        if let Ok(data) = x.data.try_into() {
          ret.push(BleUuid::Uuid128(data));
        }
      }
      _ => {}
    });
    ret
  }

  pub fn name(&self) -> Option<&BStr> {
    let data = self.decode().find(|x| {
      x.ty == (sys::BLE_HS_ADV_TYPE_COMP_NAME as _)
        || x.ty == (sys::BLE_HS_ADV_TYPE_INCOMP_NAME as _)
    })?;
    Some(BStr::new(data.data))
  }

  pub fn tx_power(&self) -> Option<u8> {
    let data = self
      .decode()
      .find(|x| x.ty == (sys::BLE_HS_ADV_TYPE_TX_PWR_LVL as _))?;

    data.data.first().copied()
  }

  pub fn service_data(&self) -> Option<BLEServiceData> {
    for x in self.decode() {
      match x.ty as u32 {
        sys::BLE_HS_ADV_TYPE_SVC_DATA_UUID16 => {
          if let Some((uuid, service_data)) = x.data.split_at_checked(2) {
            let uuid = BleUuid::from_uuid16(u16::from_le_bytes(uuid.try_into().unwrap()));
            return Some(BLEServiceData { uuid, service_data });
          } else {
            ::log::error!("Length too small for BLE_HS_ADV_TYPE_SVC_DATA_UUID16");
          }
        }
        sys::BLE_HS_ADV_TYPE_SVC_DATA_UUID32 => {
          if let Some((uuid, service_data)) = x.data.split_at_checked(4) {
            let uuid = BleUuid::from_uuid32(u32::from_le_bytes(uuid.try_into().unwrap()));
            return Some(BLEServiceData { uuid, service_data });
          } else {
            ::log::error!("Length too small for BLE_HS_ADV_TYPE_SVC_DATA_UUID32");
          }
        }
        sys::BLE_HS_ADV_TYPE_SVC_DATA_UUID128 => {
          if let Some((uuid, service_data)) = x.data.split_at_checked(16) {
            let uuid = BleUuid::from_uuid128(uuid.try_into().unwrap());
            return Some(BLEServiceData { uuid, service_data });
          } else {
            ::log::error!("Length too small for BLE_HS_ADV_TYPE_SVC_DATA_UUID128");
          }
        }
        _ => {}
      }
    }
    None
  }

  pub fn manufacture_data(&self) -> Option<ManufactureData> {
    let data = self
      .decode()
      .find(|x| x.ty == (sys::BLE_HS_ADV_TYPE_MFG_DATA as _))?;

    let (id, payload) = data.data.split_at_checked(2)?;
    Some(ManufactureData {
      company_identifier: u16::from_le_bytes(id.try_into().unwrap()),
      payload,
    })
  }

  pub fn test(&self) -> impl Iterator<Item = &u8> {
    self.payload().iter().filter(|x| (**x) != 2)
  }

  fn decode(&self) -> AdStructureIter<'_> {
    AdStructureIter {
      payload: self.payload(),
    }
  }
}

impl<T: AsRef<[u8]>> core::fmt::Debug for BLEAdvertisedData<T> {
  fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
    let mut f = f.debug_struct("BLEAdvertisedData");
    // f.field("types", &self.decode().map(|x| x.ty).collect::<Vec<_>>());
    if let Some(adv_flags) = self.adv_flags() {
      f.field("adv_flags", &adv_flags);
    }
    if let Some(name) = self.name() {
      f.field("name", &name);
    }
    if let Some(tx_power) = self.tx_power() {
      f.field("tx_power", &tx_power);
    }
    if let Some(manufacture_data) = self.manufacture_data() {
      f.field("manufacture_data", &manufacture_data);
    }
    let service_uuids = self.service_uuids();
    if !service_uuids.is_empty() {
      f.field("service_uuids", &service_uuids);
    }
    if let Some(service_data) = self.service_data() {
      f.field("service_data", &service_data);
    }
    f.finish()
  }
}

#[derive(Debug, Copy, Clone)]
pub struct BLEServiceData<'a> {
  pub uuid: BleUuid,
  pub service_data: &'a [u8],
}

#[derive(Debug, Copy, Clone)]
pub struct ManufactureData<'a> {
  pub company_identifier: u16,
  pub payload: &'a [u8],
}

struct AdData<'d> {
  ty: u8,
  data: &'d [u8],
}

struct AdStructureIter<'d> {
  payload: &'d [u8],
}

impl<'d> Iterator for AdStructureIter<'d> {
  type Item = AdData<'d>;

  fn next(&mut self) -> Option<Self::Item> {
    let length = (*self.payload.first()?) as usize;
    let (data, next_payload) = self.payload.split_at_checked(1 + length)?;
    self.payload = next_payload;
    if length == 0 {
      return None;
    }
    return Some(AdData {
      ty: unsafe { *data.get_unchecked(1) },
      data: data.get(2..(length + 1)).unwrap(),
    });
  }
}