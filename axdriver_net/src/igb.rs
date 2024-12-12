use core::convert::From;
use core::{mem::ManuallyDrop, ptr::NonNull};

use alloc::{collections::VecDeque, sync::Arc};
use axdriver_base::{BaseDriverOps, DevError, DevResult, DeviceType};
use igb_driver::{IgbDevice, IgbError, IgbNetBuf, MemPool, NicDevice};
pub use igb_driver::{IgbHal, PhysAddr, INTEL_82576, INTEL_VEND};

use crate::{EthernetAddress, NetBufPtr, NetDriverOps};

extern crate alloc;

const RECV_BATCH_SIZE: usize = 64;
const RX_BUFFER_SIZE: usize = 1024;
const MEM_POOL: usize = 4096;
const MEM_POOL_ENTRY_SIZE: usize = 2048;

/// The igb NIC device driver.
///
/// `QS` is the igb queue size, `QN` is the igb queue num.
pub struct IgbNic<H: IgbHal, const QS: usize, const QN: u16> {
    inner: IgbDevice<H, QS>,
    mem_pool: Arc<MemPool>,
    rx_buffer_queue: VecDeque<NetBufPtr>,
}

unsafe impl<H: IgbHal, const QS: usize, const QN: u16> Sync for IgbNic<H, QS, QN> {}
unsafe impl<H: IgbHal, const QS: usize, const QN: u16> Send for IgbNic<H, QS, QN> {}

impl<H: IgbHal, const QS: usize, const QN: u16> IgbNic<H, QS, QN> {
    /// Creates a net igb NIC instance and initialize, or returns a error if
    /// any step fails.
    pub fn init(base: usize, len: usize) -> DevResult<Self> {
        let mem_pool = MemPool::allocate::<H>(MEM_POOL, MEM_POOL_ENTRY_SIZE)
            .map_err(|_| DevError::NoMemory)?;
        let inner = IgbDevice::<H, QS>::init(base, len, QN, QN, &mem_pool).map_err(|err| {
            log::error!("Failed to initialize igb device: {:?}", err);
            DevError::BadState
        })?;

        let rx_buffer_queue = VecDeque::with_capacity(RX_BUFFER_SIZE);
        Ok(Self {
            inner,
            mem_pool,
            rx_buffer_queue,
        })
    }
}

impl<H: IgbHal, const QS: usize, const QN: u16> BaseDriverOps for IgbNic<H, QS, QN> {
    fn device_name(&self) -> &str {
        self.inner.get_driver_name()
    }

    fn device_type(&self) -> DeviceType {
        DeviceType::Net
    }
}

impl<H: IgbHal, const QS: usize, const QN: u16> NetDriverOps for IgbNic<H, QS, QN> {
    fn mac_address(&self) -> EthernetAddress {
        EthernetAddress(self.inner.get_mac_addr())
    }

    fn rx_queue_size(&self) -> usize {
        QS
    }

    fn tx_queue_size(&self) -> usize {
        QS
    }

    fn can_receive(&self) -> bool {
        !self.rx_buffer_queue.is_empty() || self.inner.can_receive(0).unwrap()
    }

    fn can_transmit(&self) -> bool {
        // Default implementation is return true forever.
        self.inner.can_send(0).unwrap()
    }

    fn recycle_rx_buffer(&mut self, rx_buf: NetBufPtr) -> DevResult {
        let rx_buf = igb_ptr_to_buf(rx_buf, &self.mem_pool)?;
        drop(rx_buf);
        Ok(())
    }

    fn recycle_tx_buffers(&mut self) -> DevResult {
        self.inner
            .recycle_tx_buffers(0)
            .map_err(|_| DevError::BadState)?;
        Ok(())
    }

    fn receive(&mut self) -> DevResult<NetBufPtr> {
        if !self.can_receive() {
            return Err(DevError::Again);
        }
        if !self.rx_buffer_queue.is_empty() {
            // RX buffer have received packets.
            Ok(self.rx_buffer_queue.pop_front().unwrap())
        } else {
            let f = |rx_buf| {
                let rx_buf = NetBufPtr::from(rx_buf);
                self.rx_buffer_queue.push_back(rx_buf);
            };

            // RX queue is empty, receive from igb NIC.
            match self.inner.receive_packets(0, RECV_BATCH_SIZE, f) {
                Ok(recv_nums) => {
                    if recv_nums == 0 {
                        // No packet is received, it is impossible things.
                        panic!("Error: No receive packets.")
                    } else {
                        Ok(self.rx_buffer_queue.pop_front().unwrap())
                    }
                }
                Err(e) => match e {
                    IgbError::NotReady => Err(DevError::Again),
                    _ => Err(DevError::BadState),
                },
            }
        }
    }

    fn transmit(&mut self, tx_buf: NetBufPtr) -> DevResult {
        let tx_buf = igb_ptr_to_buf(tx_buf, &self.mem_pool)?;
        match self.inner.send(0, tx_buf) {
            Ok(_) => Ok(()),
            Err(err) => match err {
                IgbError::QueueFull => Err(DevError::Again),
                _ => panic!("Unexpected err: {:?}", err),
            },
        }
    }

    fn alloc_tx_buffer(&mut self, size: usize) -> DevResult<NetBufPtr> {
        let tx_buf = IgbNetBuf::alloc(&self.mem_pool, size).map_err(|_| DevError::NoMemory)?;
        Ok(NetBufPtr::from(tx_buf))
    }
}

impl From<IgbNetBuf> for NetBufPtr {
    fn from(buf: IgbNetBuf) -> Self {
        // Use `ManuallyDrop` to avoid drop `tx_buf`.
        let mut buf = ManuallyDrop::new(buf);
        // In igb, `raw_ptr` is the pool entry, `buf_ptr` is the packet ptr, `len` is packet len
        // to avoid too many dynamic memory allocation.
        let buf_ptr = buf.packet_mut().as_mut_ptr();
        Self::new(
            NonNull::new(buf.pool_entry() as *mut u8).unwrap(),
            NonNull::new(buf_ptr).unwrap(),
            buf.packet_len(),
        )
    }
}

// Converts a `NetBufPtr` to `IgbNetBuf`.
fn igb_ptr_to_buf(ptr: NetBufPtr, pool: &Arc<MemPool>) -> DevResult<IgbNetBuf> {
    IgbNetBuf::construct(ptr.raw_ptr.as_ptr() as usize, pool, ptr.len)
        .map_err(|_| DevError::BadState)
}
