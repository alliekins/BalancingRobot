


const int freewheel       = 0;
const int forward         = 1;
const int backward        = 2;
const int brake           = 3;


/*
#define MAXSPEED 1.0
 #define BUF_SIZE 25
 #define IMAX 10
 #define ZERO 353
 
 const double p =0.07;
 const double i =0.08;
 const double d = .01;
 double setpoint=15.0;
 
 const double polynomal = 1;
 
 //this works!
 
 */
 
 /*
#define MAXSPEED 1.0
#define BUF_SIZE 35
#define IMAX 10
#define ZERO 353

const double p =0.09;
// const double i =0.02;
const double i =0.06;//trying unwinding

const double d = .01;

double setpoint=19.5;

const double polynomal = 1;*///works really well with better x offset

///////////////Experimental///////////////////
#define MAXSPEED 1.0
#define BUF_SIZE 35
#define IMAX 10
#define ZERO 353

const double p =0.09;
// const double i =0.02;
const double i =0.06;//trying unwinding

const double d = .01;

double setpoint=19.5;

const double polynomal = 1;


///////////////End Experimental///////////////////

//const double p =0.098; //linear p
//const double p =0.05; //cubic P
//const double i =0.001;
//const double d =0.00;





void setup() {
  // put your setup code here, to run once:
  pinMode(14,INPUT);  
  pinMode(15,INPUT);
  pinMode(16,INPUT);

  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(8,OUTPUT);



  Serial.begin(9600);

  setMode(forward);
  Serial.println("Started!");

}



void setMode(char mode){

  switch(mode){
  case freewheel:
    digitalWrite(2,LOW);
    digitalWrite(4,LOW);

    digitalWrite(5,LOW);
    digitalWrite(7,LOW);
    break;
  case forward:
    digitalWrite(2,LOW);
    digitalWrite(4,HIGH);

    digitalWrite(5,LOW);
    digitalWrite(8,HIGH);
    break;
  case backward:
    digitalWrite(2,HIGH);
    digitalWrite(4,LOW);

    digitalWrite(5,HIGH);
    digitalWrite(8,LOW);
    break;
  case brake:
    digitalWrite(2,HIGH);
    digitalWrite(3,HIGH);

    digitalWrite(5,HIGH);
    digitalWrite(8,HIGH);
    break;

  default:
    Serial.println("error!");

  }

}


double getAngle(){

  double x=analogRead(0)-14.29;
  double y=analogRead(1)-x;
  double z=analogRead(2)-x;

  x=0;

  double len = sqrt(x*x+y*y+z*z);

  x/=len;
  y/=len;
  z/=len;

  double dx=100.0-(len/1024.0*830.0);
  if(x>0){ //make it the same sign as x
    dx*=-1.0; 
  }


  double t=atan2((double)y, (double) z)*180/3.14;


  return t;
}


void setSpeed(double speed){


  if (speed > MAXSPEED) speed = MAXSPEED;
  if (speed < -MAXSPEED) speed = -MAXSPEED;

  if(speed < 0.0){
    setMode(backward); 
  }
  else{
    setMode(forward);
  }
  if (fabs(speed)> 0.001){
    analogWrite(3,fabs(speed)*255);
    analogWrite(6,fabs(speed)*255);
  } 
  else{
    analogWrite(3,0);
    analogWrite(6,0);
  }

}


double buffer [BUF_SIZE];
double* ip=buffer;



int count = 0;


double error=0;
double lastError=0;

double integral=0;
double deriv=0;
double average=0;

double output=0;


void loop() {
  // put your main code here, to run repeatedly: 

  char buf[30];


  double theta=getAngle();
  *(ip++)=theta;


  //calculate average
  average=0;
  for (int i =0;i<BUF_SIZE;i++){
    average+=getAngle();
    delayMicroseconds(100);
    //average += buffer[i];
  }
  if (ip-buffer>=BUF_SIZE){
    ip=buffer;
  }

  average/=((float)BUF_SIZE);

  //calculate deriv



  lastError=error;
  error=setpoint-average;
  integral+=error;

  deriv=error-lastError;

  if (integral > IMAX) integral=IMAX;
  if (integral < -IMAX)integral=-IMAX;
  if (integral*error < 0) integral=0; //unwinding


  double pc= pow(error,polynomal)*p;
  double ic =(integral)*i;  
  double dc =(deriv)*d;

  output=pc+ic+dc;

  int pi=pc*255;
  int ii=ic*255;
  int di=dc*255;

  int e = error;
  int o = output*255;
  if (count++ >= 1){
    count=0;
    sprintf(buf,"e:%d\to:%d\tp:%d\ti:%d\td:%d",e,o,pi,ii,di);
    Serial.println(buf);
  }

  setSpeed(output);

  delayMicroseconds(100);
}



