//! The boundary with the page: it writes a UTF-8 request into memory it got
//! from `alloc`, calls `call` with an operation number, and reads the UTF-8
//! answer from the pointer returned, `result_len` bytes long. The answer
//! lives until the next call.

use std::cell::RefCell;

use crate::api;

thread_local! {
    static RESULT: RefCell<Vec<u8>> = const { RefCell::new(Vec::new()) };
}

/// Memory for a request of `len` bytes.
#[no_mangle]
pub extern "C" fn alloc(len: usize) -> *mut u8 {
    let mut buffer = Vec::<u8>::with_capacity(len);
    let pointer = buffer.as_mut_ptr();
    std::mem::forget(buffer);
    pointer
}

/// Returns memory from `alloc`.
///
/// # Safety
/// `pointer` and `len` must be exactly what one `alloc` call gave.
#[no_mangle]
pub unsafe extern "C" fn dealloc(pointer: *mut u8, len: usize) {
    drop(Vec::from_raw_parts(pointer, 0, len));
}

/// Runs operation `op` on the request at `pointer`; see [`api::Op`].
///
/// # Safety
/// `pointer` must hold `len` initialised bytes.
#[no_mangle]
pub unsafe extern "C" fn call(op: u32, pointer: *const u8, len: usize) -> *const u8 {
    let request = std::slice::from_raw_parts(pointer, len);
    let answer = api::handle(op, &String::from_utf8_lossy(request));
    RESULT.with(|result| {
        let mut result = result.borrow_mut();
        *result = answer.into_bytes();
        result.as_ptr()
    })
}

/// Length of the last answer, in bytes.
#[no_mangle]
pub extern "C" fn result_len() -> usize {
    RESULT.with(|result| result.borrow().len())
}
