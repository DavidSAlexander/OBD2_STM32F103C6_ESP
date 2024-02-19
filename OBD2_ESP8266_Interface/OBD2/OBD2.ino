
/*******************************************************************************
 *  [FILE NAME]   :      <OBD2.ino>                                            *
 *  [AUTHOR]      :      <David S. Alexander>                                  *
 *  [DATE CREATED]:      <Feb 16, 2024>                                        *
 *  [Description} :      <OBD2 code for ESP8266 OBD2 Debugger>                 *
 *******************************************************************************/


 /*******************************************************************************
 *                                  Includes                                   *
 *******************************************************************************/
#include <ESP8266WiFi.h>
#include <WiFiClient.h>


/*******************************************************************************
 *                             Macro Declarations                              *
 *******************************************************************************/
#define WIFISSID              "David"
#define WIFIPASSWORD          "YOUR-PASSWORD"

#define CONNECTED             HIGH
#define DISCONNECTED          LOW

#define HOSTIP                "192.168.1.3"
#define HOSTPORT              8080

#define MSGACK                0x55
#define MSGNACK               0x33

#define MAX_RETRIES           10
#define KEEP_ALIVE_INTERVAL_MS (1 * 60 * 1000) // 1 minute in milliseconds

/*******************************************************************************
 *                             Functions Declaration                           *
 *******************************************************************************/
void WiFi_Init();
bool WiFi_Connect();
bool WiFi_Status();

void TCP_Init();
void TCP_Disconnect();
bool TCP_Connect();
bool TCP_Client_Status();
void TCP_Send_Message(String Message);
bool TCP_Incoming_Message();

void Watchdog_Init();
void Watchdog_Feed();
void Watchdog_Reset();

void sendKeepAlive();

/*******************************************************************************
 *                                 Variables                                   *
 *******************************************************************************/
unsigned long lastKeepAliveTime = 0;
WiFiClient client;
char retryCount = 0;

/*******************************************************************************
 *                            Setup Function                                   *
 *******************************************************************************/
void setup()
{
  Serial.begin(115200);
  pinMode(D4, OUTPUT);
  WiFi_Init();
  TCP_Init();
  Watchdog_Init();

  if (WiFi_Connect() && TCP_Connect())
  {
    //Serial.println("Connected to WiFi and TCP Server.");
  }
  else
  {
    //Serial.println("Failed to connect to WiFi or TCP Server. Resetting...");
    ESP.restart();
  }
}

void Read_Incomming_Data()
{
  // Read and process incoming data from TCP connection
  int receivedNumber = client.readStringUntil('\n').toInt();
  if (receivedNumber > 0)
  {
    byte MSB = receivedNumber / 256;   // Decode the incomming data
    byte LSB = receivedNumber % 256;
    Serial.write(MSB);
    Serial.write(LSB);
    client.flush();
  }
}

void loop()
{
  Watchdog_Feed(); // Reset the watchdog timer

  // Check WiFi and TCP connection status
  if (!WiFi_Status() || !TCP_Client_Status())
  {
    // Reinitialize WiFi and TCP connections if not connected
    setup();
  }
  else
  {
    // Check for incoming TCP messages
    if (TCP_Incoming_Message())
    {
      // Process incoming data from TCP
      Read_Incomming_Data();
    }
    else if (Serial.available())
    {
      // If there is data available on Serial monitor, send it to TCP server
      if (!TCP_Incoming_Message())
      {
        String receivedString = Serial.readStringUntil('\n');
        digitalWrite(D4, !digitalRead(D4));
        TCP_Send_Message(receivedString);
        client.flush();
      }
      else
      {
        // If there is an ongoing TCP message, process it first
        Read_Incomming_Data();
      }
      Serial.flush();
    }

    // Send keep-alive message at regular intervals
    if (millis() - lastKeepAliveTime >= KEEP_ALIVE_INTERVAL_MS)
    {
      sendKeepAlive();
      lastKeepAliveTime = millis(); // Reset the timer
    }
  }
}

void WiFi_Init()
{
  // Initialize WiFi connection
  delay(10);
  WiFi.begin(WIFISSID, WIFIPASSWORD);
}

bool WiFi_Connect()
{
  // Attempt to connect to WiFi network
  retryCount = 0;
  while (WiFi.status() != WL_CONNECTED && retryCount < MAX_RETRIES)
  {
    Watchdog_Feed();
    delay(1000);
    retryCount++;
  }

  return WiFi.status() == WL_CONNECTED;
}

bool WiFi_Status()
{
  // Check current WiFi connection status
  return WiFi.status() == WL_CONNECTED;
}

void TCP_Init()
{
  // Initialize TCP connection
  if (client.connected())
  {
    client.stop();
  }
}

void TCP_Disconnect()
{
  // Disconnect TCP client
  if (WiFi_Status())
  {
    client.stop();
  }
}

bool TCP_Connect()
{
  // Attempt to connect to TCP server
  if (WiFi_Status())
  {
    retryCount = 0;
    while (!client.connect(HOSTIP, HOSTPORT) && retryCount < MAX_RETRIES)
    {
      Watchdog_Feed();
      delay(1000);
      retryCount++;
    }
  }
  return client.connected();
}

bool TCP_Client_Status()
{
  // Check current TCP client status
  return client.connected();
}

void TCP_Send_Message(String Message)
{
  // Send message to TCP server
  if (TCP_Client_Status())
  {
    client.println(Message);
  }
}

bool TCP_Incoming_Message()
{
  // Check for incoming message from TCP server
  if (TCP_Client_Status())
  {
    return client.available();
  }
  return false;
}

void sendKeepAlive()
{
  // Send keep-alive message to TCP server
  if (WiFi_Status() && TCP_Client_Status())
  {
    TCP_Send_Message("Live");
  }
}

void Watchdog_Init()
{
  // Initialize watchdog timer
  ESP.wdtDisable();       // Disable the default watchdog timer
  ESP.wdtEnable(WDTO_4S); // Enable a new watchdog timer with a 4-second timeout
}

void Watchdog_Feed()
{
  // Feed the watchdog timer to prevent reset
  ESP.wdtFeed();
}

void Watchdog_Reset()
{
  // Perform a reset using the watchdog timer
  // To perform a reset using the watchdog, we can use an intentional infinite loop
  while (1)
  {
    delay(1); // Adding a small delay to allow the ESP8266 to reset
  }
}
