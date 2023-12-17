//#include <Servo.h>
int pos = 0;
//Servo servo;
bool A = true;

float pressure_sensor = A0;
int pos_linact1 =12 , neg_linact1 = 10,pos_linact2 =6 , neg_linact2 =7  ;
float pos1=20,pos2=100;
float len2, height_of_module=101,len1;//height= -0.0002*pos**2 - 0.1051*pos + 478.38


void setup() {
  // put your setup code here, to run once:
  // linear actuator here
  Serial.begin(9600);
  pinMode(pos_linact1, OUTPUT);
  pinMode(neg_linact1, OUTPUT);
  pinMode(pos_linact2, OUTPUT);
  pinMode(neg_linact2, OUTPUT);
  //servo.attach(5, 500, 2500);
}
void replacement(bool forward = false){
  if (forward){
    }
   else{
    }
  }


void loop() 
  {
    while(A && pos2<=1020)
    {
    //servo.write(120);
    float height,height_final;
    // put your main code here, to run repeatedly:
    float sensorValue = analogRead(pressure_sensor);
    pos1 = analogRead(A1);
    pos2 = analogRead(A2);  
    height= -0.0002*pos1*pos1 - 0.1051*pos1 + 478.38;  
    Serial.println(height);
      if (sensorValue > 150)
          { 
           //stop linact1
           digitalWrite(pos_linact1, HIGH);
           digitalWrite(neg_linact1, HIGH);
    
    
          //complete retraction of linact2
              while(pos2<=1015)
              {
                digitalWrite(pos_linact2, LOW);
                digitalWrite(neg_linact2, HIGH);    
                pos2 = analogRead(A2);
                //len2 =scale analog read for length of linact2; 
              }
    
    
    //height=sqrt(32*32-len1*len1);
    height_final = height + height_of_module;
    A = false;
          }
          
        else
          {
            digitalWrite(pos_linact1, LOW);
            digitalWrite(neg_linact1, HIGH);
      
          }
    }
  
    
  

    while(pos2>=600)
      {
        float height,height_final;
        pos1 = analogRead(A1);
        pos2 = analogRead(A2);  
        //len1 = 3*(pos1-31)/197;
        height= -0.0002*pos1*pos1 - 0.1051*pos1 + 478.38;
        height_final = height + height_of_module;
  
  
         while (height<=height_final)//height<height_final
            {//start linact1 till the new module reaches desired height
            pos1=analogRead(A1);
            digitalWrite(pos_linact1, LOW);
            digitalWrite(neg_linact1, HIGH);
            digitalWrite(pos_linact2, HIGH);
            digitalWrite(neg_linact2, HIGH);
            
         height=-0.0002*pos1*pos1 - 0.1051*pos1 + 478.38;
         }
         
         //open lock
         //servo.write(20);
         
        while (pos2>=510)
            {//extention of linact2, to fix new module
            pos2 = analogRead(A2);
            digitalWrite(pos_linact1, HIGH);
            digitalWrite(neg_linact1, HIGH);
            digitalWrite(pos_linact2, HIGH);
            digitalWrite(neg_linact2, LOW);    
            //len2 = //scale analog read for length of linact2 while extention;
            }
            break;
}  
}
