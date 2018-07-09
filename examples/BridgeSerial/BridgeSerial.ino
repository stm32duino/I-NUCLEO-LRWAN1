/*
  Allows to relay UART data from/to a PC terminal to/from LORA shield.

  Shield uses a LPUART interface as the AT command console, the D0 and D1 pin
  is the LPUART TX and LPUART RX, and the default configuration is 115200,N,8,1

  Important note for Nucleo64:
  by default, D0/D1 of CN9 board connector are respectively not connected to
  PA3 and PA2 (SB62 and SB63 opened).
  Those pins are connected to STLink USART thanks to SB13, SB14.

  To use the shield:
    - Connect shield D0(Tx) to PC11(Rx)
    - Connect shield D1(Rx) to PC10(Tx)
  or
    - Close SB62 and SB63 to connect D0/D1 of CN9 connector to PA3 and PA2
    - Open SB13 and SB14 to disconnect PA3 and PA2 from STLink UART
  but in this case, you will have to wire STLink Rx/Tx of CN3 connector to
  another pins and update Serial instance before call `Serial.begin(115200);`
  using:
  Serial.setRx(Rx pin);
  Serial.setTx(Tx pin);
*/
HardwareSerial SerialLora(D0, D1);

void setup()
{
  Serial.begin(115200);
  SerialLora.begin(115200);
}

// The loop function runs over and over again forever
void loop()
{
  char c;

  if (SerialLora.available() > 0)
  {
    c = SerialLora.read();
    Serial.print(c);
  }
  if (Serial.available() > 0)
  {
    c = Serial.read();
    SerialLora.print(c);
  }
}
