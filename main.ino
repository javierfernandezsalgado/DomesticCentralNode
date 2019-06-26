/*The central node reads the perifical nodes status.

  It keeps a list of the perifericals that management.

  The list can be update via telecommands

  It can be management the switch On/Off lights

  It can be get the temperatures from the nodes.*/




#include "ESP8266WiFi.h"
#include <stdint.h>
#include <time.h>
#include <Ticker.h>
#include <stdlib.h>


#define MAX_NODE_ELEMENTS (5u)
#define MAX_BUFFER_RECORD (86400u)
#define MAX_LEN_ADDRESS   (30u)
#define MAX_NAME_NET      (200u)
#define MAX_PASSW      (256u)
#define EPROOM_KEY     (4242u)
#define ERROR_DELAY    (10000u)
#define CYCLE_DELAY    (10000u)
#define TM_MAX         (3000u)


typedef enum
  {
   TEMPS=0,
   RELE=1,
   NODE=2,
   CENTRAL=3
   

  }Commands;

typedef enum
  {
   GETALLTEMPS=0,
   GETALLTEMPSMINUTES =1,

   GETTEMP = 2,
   GETTEMPMINUTES=3
   

  }TempSubCommands;

typedef enum
  {
   GETSTATUS=0,
   GETALLSTATUS =1,

   GETSWITCHON=2,
   GETSWITCHALLSWITCHON=3,

   SETRELE =4
  }ReleSubCommands;


typedef enum
  {
   GETNUMBERNODES=0,
   GETALLNODEINFO=1,
   GETNODEINFO=2,
   SETNODEINFO=3
  }nodeSubcommands;


typedef enum
  {
   SETINFOCENTRAL=0

  }centralSubCommands;


  typedef struct
   {
     time_t  timeStapm;
     float_t temperature;

  } TempRecord_st;


typedef struct
{
  uint8_t isSwitchOn;
  uint8_t status;


} Rele_st;


typedef struct
{
  uint8_t name[4u];
  uint8_t status;
  uint8_t isSwitchAvalaible;
  uint8_t isTermistorAvalaible;
  uint8_t isAvalaible;
  TempRecord_st record[MAX_BUFFER_RECORD];
  Rele_st rele;
} PerifericalNode_st;




typedef struct
{
  uint8_t address       [MAX_LEN_ADDRESS];
  uint8_t mask          [MAX_LEN_ADDRESS];
  uint8_t nameNetConnect[MAX_NAME_NET];
  uint8_t passWordNet   [MAX_PASSW];
}NodeCentralConfiguration;


typedef struct
{
   const char  *ssid;
   const char  *passw;
  uint16_t            tcp_port;

}WifiConnection_st;


static PerifericalNode_st gListPNode[MAX_NODE_ELEMENTS];
static uint8_t            gNumberNodes;
static NodeCentralConfiguration gWifiConfiguration; 
static uint8_t gInitStatus;
static Ticker gCommanderTicker;
static Ticker gHKTicker;
static WifiConnection_st gWifiConnection =
  {
   .ssid = "",
   .passw = "",
   .tcp_port =12548
   
  };
static WiFiServer gWifiServer(12548);

/*configuration */
static uint8_t  configure_ble(void);
static uint8_t  initvalueNodes(void);
static void     configurewifi(void);
static void     configureWifiServer(void);
static void     configureTimers(void);


/*Rele Operations*/
static bool      isSwitchOnRele(uint16_t nodeIndex);
static void      setSwithcRele(uint16_t node,uint8_t status);
static uint8_t   getReleStatus(uint16_t node);
static void      getAllReleStatus(uint8_t *status,uint8_t *numberNodes);


/*Temperature operations*/
static void       get_temperature_last_n_minutes(uint8_t node,uint16_t *minutes,TempRecord_st * record );
static TempRecord_st  get_temperature(uint8_t node);
static void           get_all_temperature_last_n_minutes(uint16_t minutes,TempRecord_st *record,uint16_t * tempnumber,uint8_t * readNodes);
static void           get_temperature_all_nodes(TempRecord_st * records,uint16_t *  numberReadElements);

static void                set_new_node(PerifericalNode_st datas, uint8_t index);
  static PerifericalNode_st* get_information_node(uint16_t index);

static void send_tm_initInvalid(uint8_t result);
static void command_manager (void);


static void cycleStandar();
static void HKNodes();




  
  
void setup() {


  static bool initSerial = false;
  if (initSerial == false)
    {
      Serial.begin(115200);
      initSerial = true;    
    }


  uint8_t result = true;

  Serial.println("Configure initial Node values");
  result += initvalueNodes()  ;
  
  Serial.println("Configure ble");
  result += configure_ble() << 1u;

  Serial.println("Configure Wifi");
  configurewifi();

  Serial.println("Configure Server");
  configureWifiServer();

  gInitStatus = result;
    
  if (result != 0u)
  {
    send_tm_initInvalid(result);
  }

  Serial.println("Configure Timers");
  if (result == 0u)
  {
  configureTimers() ;
    }
 

  Serial.println("Setup End");


}

void loop() {

  if (gInitStatus == 0u)
    {
      delay(CYCLE_DELAY);
    }
  else
    {
      Serial.println("Error mode");
      delay(ERROR_DELAY);
      Serial.println("Run setup again");
        setup();
      Serial.println("Send HK");
      send_tm_initInvalid(gInitStatus);
    }
  delay(1000);
}

static void     configureTimers(void)
{
  gCommanderTicker.attach_ms_scheduled(500, command_manager);
  gHKTicker.attach_ms_scheduled(5100, HKNodes);
}

static void configureWifi(void)

{


  Serial.begin(115200);
  Serial.println();

  WiFi.begin(gWifiConnection.ssid, gWifiConnection.passw);

  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      Serial.print(".");
    }
  Serial.println();

  Serial.print("Connected, IP address: ");
  Serial.println(WiFi.localIP());

}


static void configureWifiServer(void)
{
  Serial.println("The wifi server is up");
  gWifiServer.begin();
};

static void command_manager (void)
{

  /*Verify  connections*/
  WiFiClient client = gWifiServer.available();
  uint16_t counter = 0u;
  uint8_t  command [20u];
  bool     isCompleteCommand = false;
  bool     isInvalidCommand = false;

  uint8_t tm[TM_MAX];

  if (client)
    {
      Serial.print("New connection");
      while (client.connected() && !isCompleteCommand && !isInvalidCommand)
        {
          if(client.available())
            {
              command[counter] = client.read();
              if (command[counter] =='\n')
                {
                  isCompleteCommand = true;
                }
              else
                {
                  counter++;
                }
              if (counter >= 20u )
                {
                  /*Error and disconnect*/
                  isInvalidCommand = true;
                }
              
  

            }
        }
      if(isCompleteCommand && !isInvalidCommand)
        {

          switch(atoi((const char *)&command[0]))
            {

            case TEMPS:
              {
                switch(atoi((const char *)&command[1]))
                  {
                  case GETALLTEMPS:
                    {

                      Serial.println("Get all temperatures command received");
                      TempRecord_st records[gNumberNodes];
                      uint16_t numberReadElements;

                      Serial.println("Get all temperatures");
                      get_temperature_all_nodes(records,&numberReadElements);

                      
                      Serial.println("Composed TM");
                      tm[0]=0xca;
                      tm[1]=0xfe;
                      
                      memcpy(&tm[2],records,sizeof(TempRecord_st)*numberReadElements);

                      Serial.println("Send Telemetry");
                      client.println((char *)tm);
                      break;
                    }
                    
                  case GETALLTEMPSMINUTES:
                    {
                      Serial.println("Get all temperatures minutes command received");
                      uint16_t minutes;

                      memcpy(&minutes,&command[2],sizeof(uint16_t));

                      TempRecord_st records[gNumberNodes][minutes];
                      uint16_t      tempnumber[gNumberNodes];
                      uint8_t       readNodes;
                      uint8_t numberReadElements;

                      Serial.println("Get all temperatures per minute");

                      get_all_temperature_last_n_minutes(minutes,(TempRecord_st *)records,tempnumber,&readNodes);
                      
                      
                      Serial.println("Composed TM");
                      tm[0]=0xca;
                      tm[1]=0xfe;
                      
                      memcpy(&tm[2],records,sizeof(TempRecord_st)*readNodes*minutes);

                      Serial.println("Send Telemetry");
                      client.println((char *) tm);

                      
                      break;
                  

                    }
                    
                  case GETTEMP:
                    {
                      uint8_t       node;
                      TempRecord_st record;
                      uint16_t      minutes;
                      
                      node = command[2];

                      record = get_temperature(node);

                      Serial.println("Composed TM");
                      
                      tm[0]=0xca;
                      tm[1]=0xfe;
                      
                      memcpy(&tm[2],&record,sizeof(TempRecord_st));
                      
                      
                      break;
                    }
                  case GETTEMPMINUTES:
                    {

                      uint8_t       node;               
                      uint16_t      minutes;
                      TempRecord_st record[minutes];

                      
                      node = command[2];
                      memcpy(&minutes,&command[3],sizeof(uint16_t));

                      get_temperature_last_n_minutes(node,&minutes,record );



                      Serial.println("Composed TM");
                      
                      tm[0]=0xca;
                      tm[1]=0xfe;
                      
                      memcpy(&tm[2],record,sizeof(TempRecord_st)*minutes);
                                    
                      break;
                    }
                  default:
                    {
                    
                    }

                  }
                break;
              }
            case RELE:
              {
                switch(atoi((const char *)&command[1]))
                  {
                  case GETSTATUS:
                    {
                      uint8_t node= atoi((const char *)&command[2]);
                      uint8_t status;
                      
                      status = getReleStatus(node);

                      tm[0]=0xca;
                      tm[1]=0xfe;
                      tm[2] = status;
                      tm[3] = '\0';

                      client.print((char *)tm);
                    
                      break;
                    }
                  case   GETALLSTATUS:
                    {

                     // getAllReleStatus(uint8_t *status,uint8_t *numberNodes);
                      
                      break;
                    }
                  case   GETSWITCHON:
                    {
                      break;
                    }
                  case   GETSWITCHALLSWITCHON:
                    {
                      break;
                    }

                  case   SETRELE:
                    {
                      break;
                      
                    }
                  default:
                    {
                    
                    }
                  }

                break;
              }
            case NODE:
              {
                switch(atoi((const char *)&command[1]))
                  {
                  case GETNUMBERNODES:
                    {
                      break;
                    }
                    
                  case  GETALLNODEINFO:
                    {
                      break;
                    }

                  case GETNODEINFO:
                    {
                      break;
                    }
                  case SETNODEINFO:
                    {
                      break;
                    }
                  default:
                    {
                    
                    }
                  
                  }
                
                break;
              }
            case CENTRAL:
              {
                if (command[1] == SETINFOCENTRAL)
                  {

                  }
                else
                  {
                    
                  }
                break;
              }

            default:
              {
                Serial.println("Invalid Command");
                  }

            }

        }

    }




}
