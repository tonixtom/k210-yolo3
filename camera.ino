int n;
String inString = "";
char* objects[] = {"aeroplane","bycicle","bird","boat","bottle","bus","car","cat","chair","cow","diniing table","dog","horse","motorbike","person","potted plant","sheep","sofa","train","screen"};
void setup() {
Serial.begin(115200);
Serial1.begin(115200);
}
void loop() {
 if (Serial1.available() > 0) {
    int inChar = Serial1.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char and add it to the string:
      inString += (char)inChar;
    }
    // if you get a newline, print the string, then the string's value:
    if (inChar == '\n') {
      n = inString.toInt();
      // clear the string for new input:
      inString = "";
    Serial.println(objects[n]);
  }
 }
  if (Serial.available() > 0) {
    Serial1.write(Serial.read());
  }
}
