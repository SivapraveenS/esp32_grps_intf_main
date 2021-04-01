

#if 0
CoordIntf_uartRxProcTask(  )
        |
        \-------> COORD_IF_procBytesFromCoord(  )
                             |
                             |
                             \------> COORD_IF_procOneMsgFromCoord(   )
                                                   |
                                                   \-----> COORD_IF_checkProcPyld( )
                                                   |               |
                                                   |               |
                                                   |              \|/
                                                   |               |
                                                   \-----> COORD_INTF_procRcvdFrame(  )
                                                                   |
                                                                  \|/
                                                                   |
                                                           switch (rcvdMsgType)
                                                           {
                                                              case LPWMN_GW_MSG_TYPE_RELAY_FROM_NODE:
                                                                   xQueueSend(Queue2, ...);
                                                                   break;

                                                              case UART_MSG_TYPE_ACK:
                                                                   xQueueSend(Queue5, ...); 
                                                                   break;        

                                                              default:
                                                                   xQueueSend(Queue5, ...);
                                                                   break;
                                                           }



CoordIntf_procCoordMsg( )       



---------------------------------------------------------------------------------------------------------
WiFi
---------------------------------------------------------------------------------------------------------
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  int n = WiFi.scanNetworks();
  if (n == 0) 
  {
     Serial.println("no networks found");
  } 
  else 
  {
    Serial.print(n);
    Serial.println(" networks found");
  
    for (int i = 0; i < n; ++i) 
    {
       // Print SSID and RSSI for each network found
       Serial.print(i + 1);
       Serial.print(": ");
       Serial.print(WiFi.SSID(i));
       Serial.print(" (");
       Serial.print(WiFi.RSSI(i));
       Serial.print(")");
       Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*");
       delay(10);
    }
  
    Serial.println("");  
    WiFi.begin(WIFI_ssid, WIFI_pswd);

    while (WiFi.status() != WL_CONNECTED)   
    {
       Serial.printf("WS:%d NC !!\n", (int)(WiFi.status()));
       delay(500);
       Serial.print(".");
    }
  }

  Serial.printf("WS:%d Connected \n", (int)(WiFi.status()));
  Serial.printf("WiFi IP: "); 
  Serial.println(WiFi.localIP());
---------------------------------------------------------------------------------------------------------
                      
#endif                                       
