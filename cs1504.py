#!/usr/bin/env python
# Copyright (c) 2010, Bjoern Heller. All rights reserved
# This code is licensed under GNU/ GPL
import sys, time, datetime, serial, struct, pprint #global variables

#declare serial com port (may change under macosx
if sys.platform == 'darwin':
  serial_port = 'cu.usbserial-00402126'
elif sys.platform == 'linux':
  serial_port = '/dev/ttyUSB0'
elif sys.platform == 'win32':
  # this port varies from PC to PC
  serial_port = 'COM8'
else:
  serial_port = 0

version = '$Id: cs1504.py,v 2.0 15/03/2009 05:47:42 majid Exp majid $' #version string

print >> sys.stderr, ''
print >> sys.stderr, 'Hellercom.de Symbol CS1504 Barcode Scanner Software'
print >> sys.stderr, '---------------------------------------------------'
print >> sys.stderr, ''
print >> sys.stderr, 'This software is licensed under GNU/ GPL'
print >> sys.stderr, ''
print >> sys.stderr, 'tec@hellercom.de http://www.hellercom.de'
print >> sys.stderr, ''

# Revision history:
# $Log: cs1504.py,v $
#

########################################################################
# bar code conventions

def format_isbn(isbn):
  """Produce an ISBN check digit"""
  # calculate check digit
  isbn = isbn.replace('-', '')
  assert len(isbn) >= 9 and len(isbn) <= 10
  check = 0
  for i in range(9):
    check += (10 - i) * (ord(isbn[i]) - ord('0'))
  check = -check % 11
  if check == 10:
    check = 'X'
  else:
    check = str(check)
  if len(isbn) > 9:
    assert isbn[-1] == check
  else:
    isbn = isbn + check
  # lookup ISBN specs at http://www.isbn-international.org/en/userman/chapter4.html
  #
  
  return isbn

def expand(symbology, code):
  """Expand certain types of common book codes"""
  # 10-digit ISBNs are encoded as EAN-13 with the charming fictitious country
  # code 978, a.k.a. "bookland"
  # see http://www.adams1.com/pub/russadam/isbn.html
  if symbology.startswith('EAN-13') and code.startswith('978'):
    symbology = 'ISBN'
    code = format_isbn(code[3:12])
  return symbology, code

########################################################################
# the Symbol CS 1504 protocol (ref. to PDF file) got it from symbol directly
#also some setup commands supported but not yet implemented (such as led settings an
#stay awake commands

symbologies = {
  0x16: 'Bookland',
  0x0E: 'MSI',
  0x02: 'Codabar',
  0x11: 'PDF-417',
  0x0c: 'Code 11',
  0x26: 'Postbar (Canada)',
  0x20: 'Code 32',
  0x1e: 'Postnet (US)',
  0x03: 'Code 128',
  0x23: 'Postal (Australia)',
  0x01: 'Code 39',
  0x22: 'Postal (Japan)',
  0x13: 'Code 39 Full ASCII',
  0x27: 'Postal (UK)',
  0x07: 'Code 93',
  0x1c: 'QR code',
  0x1d: 'Composite',
  0x31: 'RSS limited',
  0x17: 'Coupon',
  0x30: 'RSS-14',
  0x04: 'D25',
  0x32: 'RSS Expanded',
  0x1b: 'Data Matrix',
  0x24: 'Signature',
  0x0f: 'EAN-128',
  0x15: 'Trioptic Code 39',
  0x0b: 'EAN-13',
  0x08: 'UPCA',
  0x4b: 'EAN-13+2',
  0x48: 'UPCA+2',
  0x8b: 'EAN-13+5',
  0x88: 'UPCA+5',
  0x0a: 'EAN-8',
  0x09: 'UPCE',
  0x4a: 'EAN-8+2',
  0x49: 'UPCE+2',
  0x8a: 'EAN-8+5',
  0x89: 'UPCE+5',
  0x05: 'IATA',
  0x10: 'UPCE1',
  0x19: 'ISBT-128',
  0x50: 'UPCE1+2',
  0x21: 'ISBT-128 concatenated',
  0x90: 'UPCE1+5',
  0x06: 'ITF',
  0x28: 'Macro PDF'
  }
MAX_RESP = 6144

class CS1504: #comm code

  def __init__(self, port='/dev/cu.usbserial'):
    attempts = 0
    connected = False
    while not connected:
      try:
        attempts += 1
        self.ser = serial.Serial(port,
                                 baudrate=9600,
                                 bytesize=8,
                                 parity=serial.PARITY_ODD,
                                 stopbits=serial.STOPBITS_ONE,
                                 timeout=2)
        connected = True
      except serial.SerialException:
        if attempts <= 3:
          print >> sys.stderr, 'connection on', port, 'failed, retrying'
          time.sleep(2.0)
        else:
          print >> sys.stderr, 'giving up :( bye'
          print >> 'try changing the com port settings'
          print >> 'or check your scanner for battery'
          raise
    self.delta = datetime.timedelta(0)
    self.serial = None
    self.sw_ver = None
    self.last_barcodes = []

  def interrogate(self):
    """Initiate communications with the scanner"""
    print >> sys.stderr, 'Using serial device:', self.ser.portstr + '... ',
    count = 0
    while count < 50:
      self.send('\x01\x02\x00')
      try:
        data = self.recv(23)
      except AssertionError:
        time.sleep(1.0)
        data = None
      if not data:
        count += 1
        time.sleep(0.2)
        continue
      print >> sys.stderr, 'connected'
      break
    if not data:
      raise IOError
    version, status = map(ord, data[2:4])
    assert status in [0, 22]
    if status == 22:
      print >> sys.stderr, '!!!!Scanner Battery is low!!!!'
    self.serial = data[4:12]
    self.sw_ver = data[12:20]
    assert data[20] == '\0'
    print >> sys.stderr, 'serial#', self.serial.encode('hex')
    print >> sys.stderr, 'Scanner Software version:', self.sw_ver

  def get_time(self):
    """Getting the time set in the scanner and calculating the drift..."""
    print >> sys.stderr, 'reading clock for drift...'
    self.send('\x0a\x02\x00')
    self.time_response(True)

  def set_time(self):
    """clearing scanner time..."""
    now = list(datetime.datetime.now().timetuple()[0:6])
    now[0] -= 2000
    now.reverse()
    self.send('\x09\x02\x06' + ''.join(map(chr, now)) + '\0')
    self.time_response()
    print >> sys.stderr, 'done!'

  def time_response(self, calculate_drift=False):
    now = datetime.datetime.now()
    data = self.recv(12)
    assert data[2] == '\x06'
    s, mi, h, d, m, y = map(ord, data[3:9])
    y += 2000
    ts = datetime.datetime(y, m, d, h, mi, s)
    # determine the clock drift so we can correct timestamps
    if calculate_drift:
      self.delta = now - ts
      print >> sys.stderr, 'clock drift is:', self.delta
      if abs(self.delta).seconds > 60:
        print >> sys.stderr, '!!!!Found big difference between scanner RTC and host clock!!!!',
        print >> sys.stderr, self.delta

  def get_barcodes(self):
    """Retrieving bar codes and timestamps from scanner's memory, and
    correcting clock drift...
    """
    print >> sys.stderr, 'reading barcodes...',
    count = 0
    # retry up to 5 times to read scanner
    while count < 5:
      try:
        self.send('\x07\x02\x00')
        data = self.recv()
        assert data[2:10] == self.serial, data[2:10].encode('hex')
        break
      except AssertionError:
        count += 1
        time.sleep(0.2)
    self.last_barcodes = []
    data = data[10:-3]
    while data:
      length = ord(data[0])
      first, data = data[1:length+1], data[length+1:]
      symbology = symbologies.get(ord(first[0]), 'UNKNOWN')
      code = first[1:-4]
      t = struct.unpack('>I', first[-4:])[0]
      y = 2000 + int(t & 0x3f)
      t >>= 6
      m = int(t & 0x0f)
      t >>= 4
      d = int(t & 0x1f)
      t >>= 5
      h = int(t & 0x1f)
      t >>= 5
      mi = int(t & 0x3f)
      t >>= 6
      s = int(t & 0x3f)
      ts = datetime.datetime(y, m, d, h, mi, s) + self.delta
      symbology, code = expand(symbology, code)
      self.last_barcodes.append((symbology, code, ts))
    print >> sys.stderr, 'done (%d read)' % len(self.last_barcodes)
    return self.last_barcodes

  def clear_barcodes(self):
    """Clearing the bar codes in the scanner's memory..."""
    print >> sys.stderr, 'clearing barcodes...',
    self.send('\x02\x02\x00')
    data = self.recv(5)
    print >> sys.stderr, 'done!'
      
  def power_down(self):
    """Shutting the scanner down to save battery life..."""
    print >> sys.stderr, 'powering down scanner...',
    self.send('\x05\x02\x00')
    data = self.recv(5)
    print >> sys.stderr, 'done!'
      
  def send(self, cmd):
    """Sending a command to the scanner..."""
    self.ser.write(cmd)
    self.ser.write(crc16(cmd))

  def recv(self, length=MAX_RESP):
    """Receive a response. For fixed-size responses, specifying it will take
    less time as we won't need to wait for the timeout to return data
    """
    data = self.ser.read(length)
    if data:
      assert data.startswith('\x06\x02'), data.encode('hex')
      assert data[-2:] == crc16(data[:-2])
      assert data[-3] == '\0'
    return data

  def close(self):
    self.ser.close()

  def __del__(self):
    self.close()
    del self.ser

########################################################################
# Modified from:
# http://news.hping.org/comp.lang.python.archive/18112.html
# to use the algorithm as specified by Symbol
# original crc16.py by Bryan G. Olson, 2005
# This module is free software and may be used and
# distributed under the same terms as Python itself.
import array
def crc16(string, value=0):
  """CRC function using Symbol's specified algorithm
  """
  value = 0xffff
  for ch in string:
    value = table[ord(ch) ^ (value & 0xff)] ^ (value >> 8)
  #return value
  return struct.pack('>H', ~value) #here i get an error ->get it fixed

# CRC-16 poly: p(x) = x**16 + x**15 + x**2 + 1
# top bit implicit, reflected
poly = 0xa001
table = array.array('H')
for byte in range(256):
     crc = 0
     for bit in range(8):
         if (byte ^ crc) & 1:
             crc = (crc >> 1) ^ poly
         else:
             crc >>= 1
         byte >>= 1
     table.append(crc)

assert crc16('\x01\x02\x00') == '\x9f\xde', \
       map(hex, map(ord, crc16('\x01\x02\x00')))

if __name__ == '__main__':
  scanner = CS1504(serial_port)
  scanner.interrogate()
  scanner.get_time()
  scanner.set_time()
  barcodes = scanner.get_barcodes()
  for symbology, code, timestamp in barcodes:
    print '%s,%s,%s' % (symbology, code, str(timestamp).split('.')[0])
  if barcodes:
    scanner.clear_barcodes()
  scanner.power_down()
  print >> sys.stderr, 'good bye!'
  print >> sys.stderr, ''
  print >> sys.stderr, '----------------'
