// +build windows

package serial

import (
	"fmt"
	"os"
	"sync"
	"time"
	"unsafe"

	"golang.org/x/sys/windows"
)

type Port struct {
	f  *os.File
	fd windows.Handle
	rl sync.Mutex
	wl sync.Mutex
	ro *windows.Overlapped
	wo *windows.Overlapped
}

type structDCB struct {
	DCBlength, BaudRate                            uint32
	flags                                          [4]byte
	wReserved, XonLim, XoffLim                     uint16
	ByteSize, Parity, StopBits                     byte
	XonChar, XoffChar, ErrorChar, EofChar, EvtChar byte
	wReserved1                                     uint16
}

type structTimeouts struct {
	ReadIntervalTimeout         uint32
	ReadTotalTimeoutMultiplier  uint32
	ReadTotalTimeoutConstant    uint32
	WriteTotalTimeoutMultiplier uint32
	WriteTotalTimeoutConstant   uint32
}

func openPort(name string, baud int, databits byte, parity Parity, stopbits StopBits, readTimeout time.Duration) (p *Port, err error) {
	if len(name) > 0 && name[0] != '\\' {
		name = "\\\\.\\" + name
	}

	h, err := windows.CreateFile(windows.StringToUTF16Ptr(name),
		windows.GENERIC_READ|windows.GENERIC_WRITE,
		0,
		nil,
		windows.OPEN_EXISTING,
		windows.FILE_ATTRIBUTE_NORMAL|windows.FILE_FLAG_OVERLAPPED,
		0)
	if err != nil {
		return nil, err
	}
	f := os.NewFile(uintptr(h), name)
	defer func() {
		if err != nil {
			f.Close()
		}
	}()

	if err = setCommState(h, baud, databits, parity, stopbits); err != nil {
		return nil, err
	}
	if err = setupComm(h, 64, 64); err != nil {
		return nil, err
	}
	if err = setCommTimeouts(h, readTimeout); err != nil {
		return nil, err
	}
	if err = setCommMask(h); err != nil {
		return nil, err
	}

	ro, err := newOverlapped()
	if err != nil {
		return nil, err
	}
	wo, err := newOverlapped()
	if err != nil {
		return nil, err
	}
	port := new(Port)
	port.f = f
	port.fd = h
	port.ro = ro
	port.wo = wo

	return port, nil
}

func (p *Port) Close() error {
	err := p.f.Close()
	windows.CloseHandle(p.ro.HEvent)
	windows.CloseHandle(p.wo.HEvent)
	return err
}

func (p *Port) Write(buf []byte) (int, error) {
	p.wl.Lock()
	defer p.wl.Unlock()

	if err := resetEvent(p.wo.HEvent); err != nil {
		return 0, err
	}
	var n uint32
	err := windows.WriteFile(p.fd, buf, &n, p.wo)
	if err != nil && err != windows.ERROR_IO_PENDING {
		return int(n), err
	}
	return getOverlappedResult(p.fd, p.wo)
}

func (p *Port) Read(buf []byte) (int, error) {
	if p == nil || p.f == nil {
		return 0, fmt.Errorf("Invalid port on read")
	}

	p.rl.Lock()
	defer p.rl.Unlock()

	if err := resetEvent(p.ro.HEvent); err != nil {
		return 0, err
	}
	var done uint32
	err := windows.ReadFile(p.fd, buf, &done, p.ro)
	if err != nil && err != windows.ERROR_IO_PENDING {
		return int(done), err
	}
	return getOverlappedResult(p.fd, p.ro)
}

// Discards data written to the port but not transmitted,
// or data received but not read
func (p *Port) Flush() error {
	return purgeComm(p.fd)
}

var (
	nSetCommState,
	nSetCommTimeouts,
	nSetCommMask,
	nSetupComm,
	nGetOverlappedResult,
	nPurgeComm *windows.LazyProc
)

func init() {
	k32 := windows.NewLazySystemDLL("kernel32.dll")

	nSetCommState = k32.NewProc("SetCommState")
	nSetCommTimeouts = k32.NewProc( "SetCommTimeouts")
	nSetCommMask = k32.NewProc( "SetCommMask")
	nSetupComm = k32.NewProc( "SetupComm")
	nGetOverlappedResult = k32.NewProc( "GetOverlappedResult")
	nPurgeComm = k32.NewProc( "PurgeComm")
}

func setCommState(h windows.Handle, baud int, databits byte, parity Parity, stopbits StopBits) error {
	var params structDCB
	params.DCBlength = uint32(unsafe.Sizeof(params))

	params.flags[0] = 0x01  // fBinary
	params.flags[0] |= 0x10 // Assert DSR

	params.BaudRate = uint32(baud)

	params.ByteSize = databits

	switch parity {
	case ParityNone:
		params.Parity = 0
	case ParityOdd:
		params.Parity = 1
	case ParityEven:
		params.Parity = 2
	case ParityMark:
		params.Parity = 3
	case ParitySpace:
		params.Parity = 4
	default:
		return ErrBadParity
	}

	switch stopbits {
	case Stop1:
		params.StopBits = 0
	case Stop1Half:
		params.StopBits = 1
	case Stop2:
		params.StopBits = 2
	default:
		return ErrBadStopBits
	}

	r, _, err := nSetCommState.Call(uintptr(h), uintptr(unsafe.Pointer(&params)))
	if r == 0 {
		return err
	}
	return nil
}

func setCommTimeouts(h windows.Handle, readTimeout time.Duration) error {
	var timeouts structTimeouts
	const MAXDWORD = 1<<32 - 1

	// blocking read by default
	var timeoutMs int64 = MAXDWORD - 1

	if readTimeout > 0 {
		// non-blocking read
		timeoutMs = readTimeout.Nanoseconds() / 1e6
		if timeoutMs < 1 {
			timeoutMs = 1
		} else if timeoutMs > MAXDWORD-1 {
			timeoutMs = MAXDWORD - 1
		}
	}

	/* From http://msdn.microsoft.com/en-us/library/aa363190(v=VS.85).aspx

		 For blocking I/O see below:

		 Remarks:

		 If an application sets ReadIntervalTimeout and
		 ReadTotalTimeoutMultiplier to MAXDWORD and sets
		 ReadTotalTimeoutConstant to a value greater than zero and
		 less than MAXDWORD, one of the following occurs when the
		 ReadFile function is called:

		 If there are any bytes in the input buffer, ReadFile returns
		       immediately with the bytes in the buffer.

		 If there are no bytes in the input buffer, ReadFile waits
	               until a byte arrives and then returns immediately.

		 If no bytes arrive within the time specified by
		       ReadTotalTimeoutConstant, ReadFile times out.
	*/

	timeouts.ReadIntervalTimeout = MAXDWORD
	timeouts.ReadTotalTimeoutMultiplier = MAXDWORD
	timeouts.ReadTotalTimeoutConstant = uint32(timeoutMs)

	r, _, err := nSetCommTimeouts.Call(uintptr(h), uintptr(unsafe.Pointer(&timeouts)))
	if r == 0 {
		return err
	}
	return nil
}

func setupComm(h windows.Handle, in, out int) error {
	r, _, err := nSetupComm.Call(uintptr(h), uintptr(in), uintptr(out))
	if r == 0 {
		return err
	}
	return nil
}

func setCommMask(h windows.Handle) error {
	const EV_RXCHAR = 0x0001
	r, _, err := nSetCommMask.Call(uintptr(h), EV_RXCHAR)
	if r == 0 {
		return err
	}
	return nil
}

func resetEvent(h windows.Handle) error {
	return windows.ResetEvent(h)
}

func purgeComm(h windows.Handle) error {
	const PURGE_TXABORT = 0x0001
	const PURGE_RXABORT = 0x0002
	const PURGE_TXCLEAR = 0x0004
	const PURGE_RXCLEAR = 0x0008
	r, _, err := nPurgeComm.Call(
		uintptr(h),
		PURGE_TXABORT|PURGE_RXABORT|PURGE_TXCLEAR|PURGE_RXCLEAR,
		)
	if r == 0 {
		return err
	}
	return nil
}

func newOverlapped() (*windows.Overlapped, error) {
	var overlapped windows.Overlapped
	h, err := windows.CreateEvent(nil, 1, 0, nil)
	if err != nil {
		return nil, err
	}
	overlapped.HEvent = h
	return &overlapped, nil
}

func getOverlappedResult(h windows.Handle, overlapped *windows.Overlapped) (int, error) {
	var n int
	r, _, err := nGetOverlappedResult.Call(
		uintptr(h),
		uintptr(unsafe.Pointer(overlapped)),
		uintptr(unsafe.Pointer(&n)),
		1,
		)
	if r == 0 {
		return n, err
	}

	return n, nil
}
