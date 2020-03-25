/*

*** Sterownik MIDI v.1.16.02.26 ***

  Change log:
  2015-03-23: + start projektu
  2015-03-26: + zaczolem walke z funkcja do obslugi pedalu ekspresji *nie moge ogarnac zeby wysylala komunikat tylko jak sie zmienia polozenie pedalu a nie caly czas
  2015-03-27: - rezygnacja z uzywania biblioteki midi.h  drobne zmiany w buttonpress
  2015-03-28: + dorobione menu z uzyciem switch..case i
                ciagla walka z funkcja buttonpress, niechce mi zapamietywac stanow przyciskow i ledy migaja jak chca
  2015-05-16: + zmienilem funkcje potek(); wejscie analogowe dziala teraz jak nalezy *jak narazie jako jedyna rzecz
              + dodalem opcje debugowania przez serial port
              + dodalem opcje wyboru czy sygnal bedzie wysylany przez MIDI czy USB
              + wysylanie komunikatow powinno teraz byc git bo wysylaja sie jako bity a nie cyfry (Serial.write(); zamiast Serial.print(,BYTE);
              - usunalem funkcjie buttonpress, trza  to inaczej ogarnac
  2015-09-09  + dodalem obsluge przyciskow przez biblioteke OneButton, main loop wyglada schludnie i program napewno dziala szybciej ;).
              + dodalem funkcje zapalania i gaszenia leda bez uzycia delay z przykladow np. czy dany efekt jest wlaczony( wiem ze sterownik nie zczyta co sie dzieje w dawie i niepodstawi odpowiednio ledow do faktycznego stanu wl/ wyl efektow, ale i tak fajnie miec bajer)
              - rezygnacja z menu wyboru switch..case na rzecz biblioteki OneButton
  2015-09-10  + dodana opcja do zmiany trybu pracy z wysylania wartosci 127->127->127 na 127->0->127 ledy sie resetuja podczas tego procesu.
  2015-09-19  + dluzsze przytrzymanie przycisku 3 i 4 wysyla komunikaty -1 i +1 program change
              + przeniesienie kodu odpowiedzialnego za wysylanie komunikatow w osobna funkcje tak zeby latwiej sie to wpisywalo w funkcje ktora sie odpala po odpowiednim przycisnieciu guziora i docelowo ma mi to ulatwic dolozenie if..else odpowiedzialnego za zmiane pomiedzy CC a PC
  2015-09-20  + dodalem miganie ledami zeby wiedziec kiedy sterownik jest juz gotowy do pracy
              + zrobilem animacje przy zmianie trybu pracy CC
              + wrzucilem ledy jako const na poczatku zeby potem latwo to zmienic jak juz zrobie obudowe i pozmieniam szyk polaczen na bardziej logiczny
  2015-11-08  + dodana obslugag przelacznika MIDI<->USB
  2015-12-20  + dolozona obsluga 6 przyciskow
              + dolozona obsluga ledow dla trybu pracy sterownika
              + dodana funkcja sprawdzajaca czy jest wpiety exp. pedal na podstawie odczytu pinu 12 a nie jak wczesniej funkcja potek() ze byla na chodzie caly czas
              + dodana obsluga 6 bankow i pokazywanie ktory jest wybrany mruganiem ledow przypisanych do przyciskow
              + dodana obsluga wysylania komunikatow PC
  2016-02-15  + teraz bank 6 w trybie chwilowym wysyla jedakowy numer CC ale zmienia sie jego wartosc od 0 do 5 na kolejnych przyciskach tak zeby mozna bylo kozystac z liveConfig w Reaperze
  2016-02-26  + dodalem kalibracje pedalu ekspresji ktora zapisuje w pamieci arduino wartosci min i maks. Dziala tylko jesli pedal exp jest podpiety.

  DO ZROBIENIA:
                napisac soft to edycji komunikatow sterownika z poziomu PC tak zeby sterownik zapisywal w eepromie wartosci dla danego przycisku a soft to mogl zmienic

   Funkcje:
              przytrzymujac przycisk 1 tryb CC chwilowego
              przytrzymujac przycisk 2 tryb PC
              przytrzymujac przycisk 3 tryb CC stompbox
              przytrzymujac przycisk 4 kalibracja pedalu ekspresji i zapis w pamieci min i max wychylenia pedalu. Ustawic pedal w pozycji min. wcisnac przycisk 4 poczekac 2s ustawic pedal w pozycji maks. i puscic przycisk 4.
              przytrzymujac przycisk 5 bank up + mrugniecie diody od 1-6 odpowiadajacej bankowi
              przytrzymujac przycisk 6 bank down + mrugniecie diody od 1-6 odpowiadajacej bankowi
              przelacznik z tylu zmiana wyjsc pomiedzy midi a usb
    INFO:
    d2,3,4,5,6,7, ledy przyciskow
    d8,9,10, ledy trybu
    d11 prezelacznik midi
    d12 sprawdzanie czy jest exp
    a7 exp in
    d13,14,15,16,17,18 - przyciski
*/

//#define DEBUG  // uruchomnienie debugerra ktory zamiast wysylac komunikaty MIDI bedzie wysylal informacje przez serial port
#include <OneButton.h>
#include <EEPROMex.h>



const int anPin = 7; // pedal exp jest podpiety pod wejsce analogowe nr 7
const int klaw_czas_klikniecia = 100;
const int klaw_czas_przytrzymania = 1500;
const int led1 = 2;
const int led2 = 3;
const int led3 = 4;
const int led4 = 5;
const int led5 = 6;
const int led6 = 7;
const int led7 = 8;
const int led8 = 9;
const int led9 = 10;
const int CCchanel = 0xB0;  // 1 kanal z 16 dla komunikatow CC
const int PCchanel = 0xC0;  // 1 kanal z 16 dla komunikatow PC


// Zmienne
OneButton klaw1(14, true); // przypisanie jakie wejscie bedzie odpowiadac zmiennej i czy wykoystuje wewnetrzny pull-up
OneButton klaw2(15, true);
OneButton klaw3(16, true);
OneButton klaw4(17, true);
OneButton klaw5(18, true);
OneButton klaw6(19, true);

int ledState[6] = {LOW, LOW, LOW, LOW, LOW, LOW}; // ustawienie wszystkich ledow na wylaczone
int bank[19] = {0x14, 0x15, 0x16, 0x17, 0x18, 0x19,/* <- CC */ 
                0x00, 0x01, 0x02, 0x03, 0x04, 0x05,/* <- PC */
				0x0B, /*<-pedal */
				0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F}; // podstawowy bank komunikaty nr CC 20,21,22,23,24,25 PC 00,01,02,03,04,05 i 11 dla exp. pedal wartosc CC dla kolejnych przyciskow wynosi 127;
long previousMillis = 0;
long interval = 100;
int ktory_bank = 1;
int jaki_tryb_pracy = 0;
int midiczyusb = 0;
int czy_jest_exp_pedal = 0;
int expmin = EEPROM.readByte(254);
int expmax = EEPROM.readInt(255);
long led_poprzedni_czas = 0;
int led_stan = LOW;

void setup() {
  pinMode(12, INPUT_PULLUP); // ustawianie pinu 12 jako wejsce ktore sprawdza czy jest podlaczony exp. pedal
  pinMode(11, INPUT_PULLUP); // ustawianie pinu 11 jako wejsce ktore sprawdza jaki tryb ustawic MIDI czy USB
  midiczyusb = digitalRead(11);  //sprawdzanie w jakiej pozycji jest przelacznik midi<->usb
  if (midiczyusb == HIGH)  //
    Serial.begin(115200); // 115200-USB
  else
    Serial.begin(31250);  // 31250-MIDI

  ///  Deklaracja poszczegolnych nazw funkcji przypisanych do klawiszy
  klaw1.attachClick(klaw1_klik);
  klaw1.attachLongPressStart(klaw1_long_start);
  klaw1.attachLongPressStop(klaw1_long_stop);
  klaw1.setClickTicks(klaw_czas_klikniecia);
  klaw1.setPressTicks(klaw_czas_przytrzymania);
  //klaw 2
  klaw2.attachClick(klaw2_klik);
  klaw2.attachLongPressStart(klaw2_long_start);
  klaw2.attachLongPressStop(klaw2_long_stop);
  klaw2.setClickTicks(klaw_czas_klikniecia);
  klaw2.setPressTicks(klaw_czas_przytrzymania);
  //klaw 3
  klaw3.attachClick(klaw3_klik);
  klaw3.attachLongPressStart(klaw3_long_start);
  klaw3.attachLongPressStop(klaw3_long_stop);
  klaw3.setClickTicks(klaw_czas_klikniecia);
  klaw3.setPressTicks(klaw_czas_przytrzymania);
  //klaw 4
  klaw4.attachClick(klaw4_klik);
  klaw4.attachLongPressStart(klaw4_long_start);
  klaw4.attachLongPressStop(klaw4_long_stop);
  klaw4.setClickTicks(klaw_czas_klikniecia);
  klaw4.setPressTicks(klaw_czas_przytrzymania);
  //klaw 5
  klaw5.attachClick(klaw5_klik);
  klaw5.attachLongPressStart(klaw5_long_start);
  klaw5.attachLongPressStop(klaw5_long_stop);
  klaw5.setClickTicks(klaw_czas_klikniecia);
  klaw5.setPressTicks(klaw_czas_przytrzymania);
  //klaw 6
  klaw6.attachClick(klaw6_klik);
  klaw6.attachLongPressStart(klaw6_long_start);
  klaw6.attachLongPressStop(klaw6_long_stop);
  klaw6.setClickTicks(klaw_czas_klikniecia);
  klaw6.setPressTicks(klaw_czas_przytrzymania);

  /// Koniec deklarowania nazw funkcji
  pinMode(2, OUTPUT); // LED do klawisza 1
  pinMode(3, OUTPUT); // LED do klawisza 2
  pinMode(4, OUTPUT); // ---||--- 3
  pinMode(5, OUTPUT); // ---||--- 4
  pinMode(6, OUTPUT); // ---||--- 5
  pinMode(7, OUTPUT); // ---||--- 6
  pinMode(8, OUTPUT); // LED trybu 1
  pinMode(9, OUTPUT); // ---||--- 2
  pinMode(10, OUTPUT); // ---||--- 3

#ifdef DEBUG
  Serial.print("Wartosc minimalna pedalu exp to: ");
  Serial.print(expmin);
  Serial.print(", a maksymalna to: ");
  Serial.println(expmax);
#endif
}

void loop() {
  czy_jest_exp_pedal = digitalRead(12); // uruchomnienie funkcji od pedalu ekspresji po jego podlaczeniu
  if (czy_jest_exp_pedal == HIGH) {  // uruchomnienie funkcji od pedalu ekspresji po jego podlaczeniu
    potek();
  }
  klaw1.tick();
  klaw2.tick();
  klaw3.tick();
  klaw4.tick();
  klaw5.tick();
  klaw6.tick();
  switch (jaki_tryb_pracy) {
    case 0:
      for (int a = led7; a < led9 + 1; a++) { // gaszenie wszystkich ledow
        digitalWrite(a, LOW);
      }
      digitalWrite(led7, HIGH);
      break;
    case 1:
      for (int a = led7; a < led9 + 1; a++) { // gaszenie wszystkich ledow
        digitalWrite(a, LOW);
      }
      digitalWrite(led8, HIGH);
      break;
    case 2:
      for (int a = led7; a < led9 + 1; a++) { // gaszenie wszystkich ledow
        digitalWrite(a, LOW);
      }
      digitalWrite(led9, HIGH);
      break;
  }
}

void potek() {
  static int s_nLastPotValue = 0;
  static int s_nLastMappedValue = 0;
  int nCurrentPotValue = analogRead(anPin);
  if (abs(nCurrentPotValue - s_nLastPotValue) < 50) // wieksze niz 7 bo jest to zabespieczenie przed blednym wysylaniem komunikatow MIDI
    return;
  s_nLastPotValue = nCurrentPotValue;
  if (nCurrentPotValue <= expmin) nCurrentPotValue = expmin; // ograniczenie zakresu pracy pedalu
  if (nCurrentPotValue >= expmax) nCurrentPotValue = expmax; // ograniczenie zakresu pracy pedalu
  int nMappedValue = map(nCurrentPotValue, expmin, expmax, 0, 127); // Map the value to 0-127
  if (nMappedValue == s_nLastMappedValue)
    return;
  s_nLastMappedValue = nMappedValue;  // zapis ostatniej pozycji pedalu

  midiSend(0xB0, bank[12], nMappedValue);  // wysyla komunikat CC na kanale zapisanym w banku (domyslnie 11 - expression pedal wg. specyfikacji MIDI) o zanej wartosci 0-127
}

//  Send a three byte midi message
void midiSend(char status, char data1, char data2) {
#ifdef DEBUG
  Serial.println("3-bitowy komunikat");
  Serial.print("Rodzaj komunikatu: ");
  if (status == -80)
    Serial.print(" CC ");
  else
    Serial.print(status, HEX);
  Serial.print(", kontroler nr: ");
  Serial.print(data1, DEC);
  Serial.print(", wartosc: ");
  Serial.print(data2, DEC);
  Serial.print(", Bank nr: ");
  Serial.println(ktory_bank);
#else
  Serial.write(status);
  Serial.write(data1);
  Serial.write(data2);
#endif
}

//  Send a two byte midi message
void midiProg(char status, int data ) {
#ifdef DEBUG
  Serial.println("2-bitowy komunikat");
  Serial.print("Rodzaj komunikatu: ");
  if (status == -64)
    Serial.print(" PC ");
  else
    Serial.print(status, DEC);
  Serial.print(", wartosc: ");
  Serial.print(data, DEC);
  Serial.print(", Bank nr: ");
  Serial.println(ktory_bank);
#else
  Serial.write(status);
  Serial.write(data);
#endif
}

void flashPin( int ledPin, int flashDelay) {
  digitalWrite( ledPin, HIGH );
  delay( flashDelay );
  digitalWrite( ledPin, LOW );
}

void tryb_chwilowy(int cc, int wart, int lout)  // cc= numer komunikatu cc, wart= wartosc dla komunikatu CC, lout= ktorym ledem blysnac
{
  midiSend( CCchanel, cc, wart);
#ifdef DEBUG
  Serial.println("Wysyla caly czas wartosc 127");
#endif
  flashPin(lout, 100);
}

void tryb_stompbox(int cc, int lout, int lm) //lm = ktory led z matrycy ledstate, cc= numer komunikatu cc, lout = na ktorym wyjsciu zapalac/gasic leda
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;    // save the last time you blinked the LED
    if (ledState[lm] == LOW) { // if the LED is off turn it on and vice-versa:
      ledState[lm] = HIGH;
#ifdef DEBUG
      Serial.println("Wysyla teraz wartosc 127");
#endif
      midiSend( CCchanel, cc, 0x7F );

    }
    else {
      ledState[lm] = LOW;
#ifdef DEBUG
      Serial.println("Wysyla teraz wartosc 0");
#endif
      midiSend( CCchanel, cc, 0x00 );

    }
    digitalWrite(lout, ledState[lm]);  // set the LED with the ledState of the variable:
  }
}
void tryb_programchange(int pc, int lout)
{
  //  #ifdef DEBUG
  //  Serial.println("Wysyla teraz komunikaty Program Change");
  //  #endif
  midiProg(PCchanel, pc);
  flashPin(lout, 100);
}

//
// W tych funkcjach wstawiac wszystkie rzeczy jakie sterownik ma zrobic po nacisnieciu klawisza
//
//////////////////////////////////////
// ------------------ Klawiwsz 1
//////////////////////////////////////
void klaw1_klik()
{
  switch (jaki_tryb_pracy) {
    case 0:
      tryb_chwilowy(bank[0], bank[13], led1); // w nawiasie numer komunikatu cc i numer leda
      break;
    case 1:
      tryb_programchange(bank[6], led1);
      break;
    case 2:
      tryb_stompbox(bank[0], led1, 0); // nr komunikatu, numer leda, pozycja leda w matrycy
      break;
  }
}

void klaw1_long_start()
{
  jaki_tryb_pracy = 0;
#ifdef DEBUG
  Serial.print("Status zmieniony na wysylanie CC 127 -> 127 -> 127 ..");
  Serial.println(jaki_tryb_pracy);
#endif
  for (int a = led1; a < led6 + 1; a++) { // gaszenie wszystkich ledow
    digitalWrite(a, LOW);
  }
  ledState[0] = LOW;  // ustawianie wszystkich ledow w stan niski zeby potem blednie sie nie zapalaly i nie wysylaly odwrotnych wartosci CC
  ledState[1] = LOW;
  ledState[2] = LOW;
  ledState[3] = LOW;
  ledState[4] = LOW;
  ledState[5] = LOW;
}
void klaw1_long_stop()
{
}
//////////////////////////////////////
// ------------------ Klawiwsz 2
//////////////////////////////////////
void klaw2_klik()
{
  switch (jaki_tryb_pracy) {
    case 0:
      tryb_chwilowy(bank[1], bank[14], led2); // w nawiasie numer komunikatu cc i numer leda
      break;
    case 1:
      tryb_programchange(bank[7], led2);
      break;
    case 2:
      tryb_stompbox(bank[1], led2, 1); // nr komunikatu, numer leda, pozycja leda w matrycy
      break;
  }
}

void klaw2_long_start()
{
  jaki_tryb_pracy = 1;
#ifdef DEBUG
  Serial.print("Wysyla teraz komunikaty Program Change, ");
  Serial.println(jaki_tryb_pracy);
#endif
  for (int a = led1; a < led6 + 1; a++) { // gaszenie wszystkich ledow
    digitalWrite(a, LOW);
  }
  ledState[0] = LOW;  // ustawianie wszystkich ledow w stan niski zeby potem blednie sie nie zapalaly i nie wysylaly odwrotnych wartosci CC
  ledState[1] = LOW;
  ledState[2] = LOW;
  ledState[3] = LOW;
  ledState[4] = LOW;
  ledState[5] = LOW;
}
void klaw2_long_stop()
{
}
//////////////////////////////////////
// ------------------ Klawiwsz 3
//////////////////////////////////////
void klaw3_klik()
{
  switch (jaki_tryb_pracy) {
    case 0:
      tryb_chwilowy(bank[2], bank[15], led3); // w nawiasie numer komunikatu cc i numer leda
      break;
    case 1:
      tryb_programchange(bank[8], led3);
      break;
    case 2:
      tryb_stompbox(bank[2], led3, 2); // nr komunikatu, numer leda, pozycja leda w matrycy
      break;
  }
}

void klaw3_long_start()
{
  jaki_tryb_pracy = 2;
#ifdef DEBUG
  Serial.print("Status zmieniony na wysylanie CC 127 -> 0 -> 127 ..");
  Serial.println(jaki_tryb_pracy);
#endif
  for (int a = led1; a < led6 + 1; a++) { // gaszenie wszystkich ledow
    digitalWrite(a, LOW);
  }
  ledState[0] = LOW;  // ustawianie wszystkich ledow w stan niski zeby potem blednie sie nie zapalaly i nie wysylaly odwrotnych wartosci CC
  ledState[1] = LOW;
  ledState[2] = LOW;
  ledState[3] = LOW;
  ledState[4] = LOW;
  ledState[5] = LOW;
}
void klaw3_long_stop()
{
}
//////////////////////////////////////
// ------------------ Klawiwsz 4
//////////////////////////////////////
void klaw4_klik()
{
  switch (jaki_tryb_pracy) {
    case 0:
      tryb_chwilowy(bank[3], bank[16], led4); // w nawiasie numer komunikatu cc, wartosc komunikatu i numer leda
      break;
    case 1:
      tryb_programchange(bank[9], led4);
      break;
    case 2:
      tryb_stompbox(bank[4], led4, 3); // nr komunikatu, numer leda, pozycja leda w matrycy
      break;
  }
}
void klaw4_long_start()
{
  czy_jest_exp_pedal = digitalRead(12); // sprawdzanie czy pedal exp jest podlaczony
  if (czy_jest_exp_pedal == HIGH) {  // sprawdzanie czy pedal exp jest podlaczony
    expmin = analogRead(anPin);  // odczytuje wartosc pedalu
    EEPROM.writeByte(254, expmin); // zapisuje ja w pamieci jako wartosc minimalna
#ifdef DEBUG
    Serial.print("Wartosc minimalna pedalu exp: ");
    Serial.println(expmin);
#endif
  }
}
void klaw4_long_stop()
{
  czy_jest_exp_pedal = digitalRead(12); // sprawdzanie czy pedal exp jest podlaczony
  if (czy_jest_exp_pedal == HIGH) {  // sprawdzanie czy pedal exp jest podlaczony
    expmax = analogRead(anPin); // odczytuje wartosc pedalu
    EEPROM.writeInt(255, expmax); // zapisuje ja w pamieci jako wartosc maksymalna
#ifdef DEBUG
    Serial.print("Wartosc maksymalna pedalu exp: ");
    Serial.println(expmax);
#endif
  }
}
//////////////////////////////////////
// ------------------ Klawiwsz 5
//////////////////////////////////////
void klaw5_klik()
{
  switch (jaki_tryb_pracy) {
    case 0:
      tryb_chwilowy(bank[4], bank[17], led5); // w nawiasie numer komunikatu cc i numer leda
      break;
    case 1:
      tryb_programchange(bank[10], led5);
      break;
    case 2:
      tryb_stompbox(bank[4], led5, 4); // nr komunikatu, numer leda, pozycja leda w matrycy
      break;
  }
}

void klaw5_long_start()
{
  ktory_bank = ktory_bank + 1 ;
  if (ktory_bank > 6) (ktory_bank = 1);
  zmiana_bankow(ktory_bank);
  for (int a = led1; a < led6 + 1; a++) { // gaszenie wszystkich ledow
    digitalWrite(a, LOW);
  }
  ledState[0] = LOW;  // ustawianie wszystkich ledow w stan niski zeby potem blednie sie nie zapalaly i nie wysylaly odwrotnych wartosci CC
  ledState[1] = LOW;
  ledState[2] = LOW;
  ledState[3] = LOW;
  ledState[4] = LOW;
  ledState[5] = LOW;
  flashPin(ktory_bank + 1, 100);
  delay(100);
  flashPin(ktory_bank + 1, 100);
  delay(100);
  flashPin(ktory_bank + 1, 100);


#ifdef DEBUG
  Serial.print("Bank nr: ");
  Serial.println(ktory_bank);
#endif
}

void klaw5_long_stop()
{
}

//////////////////////////////////////
// ------------------ Klawiwsz 6
//////////////////////////////////////
void klaw6_klik()
{
  switch (jaki_tryb_pracy) {
    case 0:
      tryb_chwilowy(bank[5], bank[18], led6); // w nawiasie numer komunikatu cc i numer leda
      break;
    case 1:
      tryb_programchange(bank[11], led6);
      break;
    case 2:
      tryb_stompbox(bank[5], led6, 5); // nr komunikatu, numer leda, pozycja leda w matrycy
      break;
  }
}


void klaw6_long_start()
{
  ktory_bank = ktory_bank - 1 ;
  if (ktory_bank < 1) (ktory_bank = 6);
  zmiana_bankow(ktory_bank);
  for (int a = led1; a < led6 + 1; a++) { // gaszenie wszystkich ledow
    digitalWrite(a, LOW);
  }
  ledState[0] = LOW;  // ustawianie wszystkich ledow w stan niski zeby potem blednie sie nie zapalaly i nie wysylaly odwrotnych wartosci CC
  ledState[1] = LOW;
  ledState[2] = LOW;
  ledState[3] = LOW;
  ledState[4] = LOW;
  ledState[5] = LOW;
  flashPin(ktory_bank + 1, 100);
  delay(100);
  flashPin(ktory_bank + 1, 100);
  delay(100);
  flashPin(ktory_bank + 1, 100);

#ifdef DEBUG
  Serial.print("Bank nr: ");
  Serial.println(ktory_bank);
#endif
}

void klaw6_long_stop()
{
}
// *****************************************************
// ***************************** B A N K I  ************
// *****************************************************
void zmiana_bankow(int ktory_bank2)
{
  switch (ktory_bank2) {
    case 1:
      bank[0] = 0x14; // CC klaw 1
      bank[1] = 0x15; // CC klaw 2
      bank[2] = 0x16; // CC klaw 3
      bank[3] = 0x17; // CC klaw 4
      bank[4] = 0x18; // CC klaw 5
      bank[5] = 0x19; // CC klaw 6
      bank[6] = 0x00; // PC klaw 1
      bank[7] = 0x01; // PC klaw 2
      bank[8] = 0x02; // PC klaw 3
      bank[9] = 0x03; // PC klaw 4
      bank[10] = 0x04; // PC klaw 5
      bank[11] = 0x05; // PC klaw 6
      bank[12] = 0x0B; // exp. pedal
      bank[13] = 0x7F; // wartosc CC dla klaw 1
      bank[14] = 0x7F; // wartosc CC dla klaw 2
      bank[15] = 0x7F; // wartosc CC dla klaw 3
      bank[16] = 0x7F; // wartosc CC dla klaw 4
      bank[17] = 0x7F; // wartosc CC dla klaw 5
      bank[18] = 0x7F; // wartosc CC dla klaw 6
      break;
    case 2:
      bank[0] = 0x1A; // CC klaw 1
      bank[1] = 0x1B; // CC klaw 2
      bank[2] = 0x1C; // CC klaw 3
      bank[3] = 0x1D; // CC klaw 4
      bank[4] = 0x1E; // CC klaw 5
      bank[5] = 0x1F; // CC klaw 6
      bank[6] = 0x06; // PC klaw 1
      bank[7] = 0x07; // PC klaw 2
      bank[8] = 0x08; // PC klaw 3
      bank[9] = 0x09; // PC klaw 4
      bank[10] = 0x1A; // PC klaw 5
      bank[11] = 0x1B; // PC klaw 6
      bank[12] = 0x0B; // exp. pedal
      bank[13] = 0x7F; // wartosc CC dla klaw 1
      bank[14] = 0x7F; // wartosc CC dla klaw 2
      bank[15] = 0x7F; // wartosc CC dla klaw 3
      bank[16] = 0x7F; // wartosc CC dla klaw 4
      bank[17] = 0x7F; // wartosc CC dla klaw 5
      bank[18] = 0x7F; // wartosc CC dla klaw 6
      break;
    case 3:
      bank[0] = 0x20; // CC klaw 1
      bank[1] = 0x21; // CC klaw 2
      bank[2] = 0x22; // CC klaw 3
      bank[3] = 0x23; // CC klaw 4
      bank[4] = 0x24; // CC klaw 5
      bank[5] = 0x25; // CC klaw 6
      bank[6] = 0x1C; // PC klaw 1
      bank[7] = 0x1D; // PC klaw 2
      bank[8] = 0x1E; // PC klaw 3
      bank[9] = 0x1F; // PC klaw 4
      bank[10] = 0x20; // PC klaw 5
      bank[11] = 0x21; // PC klaw 6
      bank[12] = 0x0B; // exp. pedal
      bank[13] = 0x7F; // wartosc CC dla klaw 1
      bank[14] = 0x7F; // wartosc CC dla klaw 2
      bank[15] = 0x7F; // wartosc CC dla klaw 3
      bank[16] = 0x7F; // wartosc CC dla klaw 4
      bank[17] = 0x7F; // wartosc CC dla klaw 5
      bank[18] = 0x7F; // wartosc CC dla klaw 6
      break;
    case 4:
      bank[0] = 0x26; // CC klaw 1
      bank[1] = 0x27; // CC klaw 2
      bank[2] = 0x28; // CC klaw 3
      bank[3] = 0x29; // CC klaw 4
      bank[4] = 0x2A; // CC klaw 5
      bank[5] = 0x2B; // CC klaw 6
      bank[6] = 0x22; // PC klaw 1
      bank[7] = 0x23; // PC klaw 2
      bank[8] = 0x24; // PC klaw 3
      bank[9] = 0x25; // PC klaw 4
      bank[10] = 0x26; // PC klaw 5
      bank[11] = 0x27; // PC klaw 6
      bank[12] = 0x0B; // exp. pedal
      bank[13] = 0x7F; // wartosc CC dla klaw 1
      bank[14] = 0x7F; // wartosc CC dla klaw 2
      bank[15] = 0x7F; // wartosc CC dla klaw 3
      bank[16] = 0x7F; // wartosc CC dla klaw 4
      bank[17] = 0x7F; // wartosc CC dla klaw 5
      bank[18] = 0x7F; // wartosc CC dla klaw 6
      break;
    case 5:
      bank[0] = 0x2C; // CC klaw 1
      bank[1] = 0x2D; // CC klaw 2
      bank[2] = 0x2E; // CC klaw 3
      bank[3] = 0x2F; // CC klaw 4
      bank[4] = 0x30; // CC klaw 5
      bank[5] = 0x31; // CC klaw 6
      bank[6] = 0x28; // PC klaw 1
      bank[7] = 0x29; // PC klaw 2
      bank[8] = 0x2A; // PC klaw 3
      bank[9] = 0x2B; // PC klaw 4
      bank[10] = 0x2C; // PC klaw 5
      bank[11] = 0x2D; // PC klaw 6
      bank[12] = 0x0B; // exp. pedal
      bank[13] = 0x7F; // wartosc CC dla klaw 1
      bank[14] = 0x7F; // wartosc CC dla klaw 2
      bank[15] = 0x7F; // wartosc CC dla klaw 3
      bank[16] = 0x7F; // wartosc CC dla klaw 4
      bank[17] = 0x7F; // wartosc CC dla klaw 5
      bank[18] = 0x7F; // wartosc CC dla klaw 6
      break;
    case 6:
      bank[0] = 0x32; // CC klaw 1
      bank[1] = 0x32; // CC klaw 2
      bank[2] = 0x32; // CC klaw 3
      bank[3] = 0x32; // CC klaw 4
      bank[4] = 0x32; // CC klaw 5
      bank[5] = 0x32; // CC klaw 6
      bank[6] = 0x2E; // PC klaw 1
      bank[7] = 0x2F; // PC klaw 2
      bank[8] = 0x30; // PC klaw 3
      bank[9] = 0x31; // PC klaw 4
      bank[10] = 0x32; // PC klaw 5
      bank[11] = 0x33; // PC klaw 6
      bank[13] = 0x00; // wartosc CC dla klaw 1
      bank[14] = 0x01; // wartosc CC dla klaw 2
      bank[15] = 0x02; // wartosc CC dla klaw 3
      bank[16] = 0x03; // wartosc CC dla klaw 4
      bank[17] = 0x04; // wartosc CC dla klaw 5
      bank[18] = 0x05; // wartosc CC dla klaw 6
      break;
  }
}
