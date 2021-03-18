

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

#endif

                                                             
