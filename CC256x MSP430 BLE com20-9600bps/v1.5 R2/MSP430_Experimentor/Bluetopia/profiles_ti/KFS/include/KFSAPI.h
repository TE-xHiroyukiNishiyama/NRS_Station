/*****< kfsapi.h >*************************************************************/
/*      Copyright 2011 - 2014 Stonestreet One.                                */
/*      All Rights Reserved.                                                  */
/*                                                                            */
/*  KFSAPI - Stonestreet One Bluetooth Key Fob Service (GATT based - TI       */
/*           Proprietary) API Type Definitions, Constants, and Prototypes.    */
/*                                                                            */
/*  Author:  Tim Cook                                                         */
/*                                                                            */
/*** MODIFICATION HISTORY *****************************************************/
/*                                                                            */
/*   mm/dd/yy  F. Lastname    Description of Modification                     */
/*   --------  -----------    ------------------------------------------------*/
/*   08/27/12  T. Cook        Initial creation.                               */
/******************************************************************************/
#ifndef __KFSAPIH__
#define __KFSAPIH__

#include "SS1BTPS.h"        /* Bluetooth Stack API Prototypes/Constants.      */
#include "SS1BTGAT.h"       /* Bluetooth Stack GATT API Prototypes/Constants. */
#include "KFSTypes.h"       /* Key Fob Service Types/Constants.               */

   /* Error Return Codes.                                               */

   /* Error Codes that are smaller than these (less than -1000) are     */
   /* related to the Bluetooth Protocol Stack itself (see BTERRORS.H).  */
#define KFS_ERROR_INVALID_PARAMETER                      (-1000)
#define KFS_ERROR_INSUFFICIENT_RESOURCES                 (-1001)
#define KFS_ERROR_SERVICE_ALREADY_REGISTERED             (-1003)
#define KFS_ERROR_INVALID_INSTANCE_ID                    (-1004)
#define KFS_ERROR_MALFORMATTED_DATA                      (-1005)
#define KFS_ERROR_UNKNOWN_ERROR                          (-1006)

   /* The following controls the maximum length of a static string.     */
#define KFS_MAXIMUM_SUPPORTED_STRING                     (248)

   /* The following function represents the container for the           */
   /* information available in the Key Pressed Characteristic.          */
typedef struct _tagKFS_Key_Pressed_State_t
{
   Boolean_t KeyOnePressed;
   Boolean_t KeyTwoPressed;
} KFS_Key_Pressed_State_t;

#define KFS_KEY_PRESSED_STATE_DATA_SIZE                  (sizeof(KFS_Key_Pressed_State_t))

   /* The following structure contains the Handles that will need to be */
   /* cached by a ACC client in order to only do service discovery once.*/
typedef struct _tagKFS_Client_Information_t
{
   Word_t Key_State_Characteristic;
   Word_t Key_State_Characteristic_CCCD;
} KFS_Client_Information_t;

#define KFS_CLIENT_INFORMATION_DATA_SIZE                (sizeof(KFS_Client_Information_t))

   /* The following structure contains all of the per Client data that  */
   /* will need to be stored by a KFS Server.                           */
typedef struct _tagKFS_Server_Information_t
{
   Boolean_t Key_Press_Client_Configuration;
} KFS_Server_Information_t;

   /* The following enumeration covers all the events generated by the  */
   /* KFS Profile.  These are used to determine the type of each event  */
   /* generated, and to ensure the proper union element is accessed for */
   /* the KFS_Event_Data_t structure.                                   */
typedef enum
{
   etKFS_Server_Read_Client_Configuration_Request,
   etKFS_Server_Client_Configuration_Update
} KFS_Event_Type_t;

   /* The following KFS Profile Event is dispatched to a KFS Server when*/
   /* a KFS Client is attempting to read a descriptor.  The             */
   /* ConnectionID, ConnectionType, and RemoteDevice specifiy the Client*/
   /* that is making the request.  The TransactionID specifies the      */
   /* TransactionID of the request, this can be used when responding to */
   /* the request using the KFS_Client_Configuration_Read_Response() API*/
   /* function.                                                         */
   /* * NOTE * This is only dispatched to read the Key Pressed Client   */
   /*          Characteristic Configuration Descriptor.                 */
typedef struct _tagKFS_Read_Client_Configuration_Data_t
{
   unsigned int              InstanceID;
   unsigned int              ConnectionID;
   unsigned int              TransactionID;
   GATT_Connection_Type_t    ConnectionType;
   BD_ADDR_t                 RemoteDevice;
} KFS_Read_Client_Configuration_Data_t;

#define KFS_READ_CLIENT_CONFIGURATION_DATA_SIZE          (sizeof(KFS_Read_Client_Configuration_Data_t))

   /* The following KFS Profile Event is dispatched to a KFS Server when*/
   /* a KFS Client has written a Client Configuration descriptor.  The  */
   /* ConnectionID, ConnectionType, and RemoteDevice specifiy the Client*/
   /* that is making the update.                                        */
   /* * NOTE * This is only dispatched to read the Key Pressed Client   */
   /*          Characteristic Configuration Descriptor.                 */
typedef struct _tagKFS_Client_Configuration_Update_Data_t
{
   unsigned int              InstanceID;
   unsigned int              ConnectionID;
   GATT_Connection_Type_t    ConnectionType;
   BD_ADDR_t                 RemoteDevice;
   Boolean_t                 NotificationsEnabled;
} KFS_Client_Configuration_Update_Data_t;

#define KFS_CLIENT_CONFIGURATION_UPDATE_DATA_SIZE        (sizeof(KFS_Client_Configuration_Update_Data_t))

   /* The following structure represents the container structure for    */
   /* holding all KFS Profile Event Data.  This structure is received   */
   /* for each event generated.  The Event_Data_Type member is used to  */
   /* determine the appropriate union member element to access the      */
   /* contained data.  The Event_Data_Size member contains the total    */
   /* size of the data contained in this event.                         */
typedef struct _tagKFS_Event_Data_t
{
   KFS_Event_Type_t Event_Data_Type;
   Word_t           Event_Data_Size;
   union
   {
      KFS_Read_Client_Configuration_Data_t   *KFS_Read_Client_Configuration_Data;
      KFS_Client_Configuration_Update_Data_t *KFS_Client_Configuration_Update_Data;
   } Event_Data;
} KFS_Event_Data_t;

#define KFS_EVENT_DATA_SIZE                             (sizeof(KFS_Event_Data_t))

   /* The following declared type represents the Prototype Function for */
   /* a KFS Profile Event Receive Data Callback.  This function will be */
   /* called whenever an KFS Profile Event occurs that is associated    */
   /* with the specified Bluetooth Stack ID.  This function passes to   */
   /* the caller the Bluetooth Stack ID, the KFS Event Data that        */
   /* occurred and the KFS Profile Event Callback Parameter that was    */
   /* specified when this Callback was installed.  The caller is free to*/
   /* use the contents of the KFS Profile Event Data ONLY in the context*/
   /* of this callback.  If the caller requires the Data for a longer   */
   /* period of time, then the callback function MUST copy the data into*/
   /* another Data Buffer This function is guaranteed NOT to be invoked */
   /* more than once simultaneously for the specified installed callback*/
   /* (i.e.  this function DOES NOT have be re-entrant).  It needs to be*/
   /* noted however, that if the same Callback is installed more than   */
   /* once, then the callbacks will be called serially.  Because of     */
   /* this, the processing in this function should be as efficient as   */
   /* possible.  It should also be noted that this function is called in*/
   /* the Thread Context of a Thread that the User does NOT own.        */
   /* Therefore, processing in this function should be as efficient as  */
   /* possible (this argument holds anyway because another KFS Profile  */
   /* Event will not be processed while this function call is           */
   /* outstanding).                                                     */
   /* ** NOTE ** This function MUST NOT Block and wait for events that  */
   /*            can only be satisfied by Receiving KFS Profile Event   */
   /*            Packets.  A Deadlock WILL occur because NO KFS Event   */
   /*            Callbacks will be issued while this function is        */
   /*            currently outstanding.                                 */
typedef void (BTPSAPI *KFS_Event_Callback_t)(unsigned int BluetoothStackID, KFS_Event_Data_t *KFS_Event_Data, unsigned long CallbackParameter);

   /* The following function is responsible for opening a KFS Server.   */
   /* The first parameter is the Bluetooth Stack ID on which to open the*/
   /* server.  The final parameter is a pointer to store the GATT       */
   /* Service ID of the registered KFS service.  This can be used to    */
   /* include the service registered by this call.  This function       */
   /* returns the positive, non-zero, Instance ID or a negative error   */
   /* code.                                                             */
BTPSAPI_DECLARATION int BTPSAPI KFS_Initialize_Service(unsigned int BluetoothStackID, KFS_Event_Callback_t EventCallback, unsigned long CallbackParameter, unsigned int *ServiceID);

#ifdef INCLUDE_BLUETOOTH_API_PROTOTYPES
   typedef int (BTPSAPI *PFN_KFS_Initialize_Service_t)(unsigned int BluetoothStackID, KFS_Event_Callback_t EventCallback, unsigned long CallbackParameter, unsigned int *ServiceID);
#endif

   /* The following function is responsible for closing a previously KFS*/
   /* Server.  The first parameter is the Bluetooth Stack ID on which to*/
   /* close the server.  The second parameter is the InstanceID that was*/
   /* returned from a successfull call to KFS_Initialize_Service().     */
   /* This function returns a zero if successful or a negative return   */
   /* error code if an error occurs.                                    */
BTPSAPI_DECLARATION int BTPSAPI KFS_Cleanup_Service(unsigned int BluetoothStackID, unsigned int InstanceID);

#ifdef INCLUDE_BLUETOOTH_API_PROTOTYPES
   typedef int (BTPSAPI *PFN_KFS_Cleanup_Service_t)(unsigned int BluetoothStackID, unsigned int InstanceID);
#endif

   /* The following function is responsible for setting the Key Pressed */
   /* User Characteristic Descriptor on the specified Key Fob Service   */
   /* instance.  The first parameter is the Bluetooth Stack ID of the   */
   /* Bluetooth Device.  The second parameter is the InstanceID returned*/
   /* from a successful call to KFS_Initialize_Server().  The final     */
   /* parameter is the User Description to set as the current User      */
   /* Description for the specified Key Fob Service Instance.  The User */
   /* Description parameter must be a pointer to a NULL terminated ASCII*/
   /* String of at most KFS_MAXIMUM_SUPPORTED_STRING (not counting the  */
   /* trailing NULL terminator).  This function returns a zero if       */
   /* successful or a negative return error code if an error occurs.    */
BTPSAPI_DECLARATION int BTPSAPI KFS_Set_Key_Press_User_Description(unsigned int BluetoothStackID, unsigned int InstanceID, char *UserDescription);
                                                                                                
#ifdef INCLUDE_BLUETOOTH_API_PROTOTYPES                                                         
   typedef int (BTPSAPI *PFN_KFS_Set_Key_Press_User_Description_t)(unsigned int BluetoothStackID, unsigned int InstanceID, char *UserDescription);
#endif
   
   /* The following function is responsible for querying the current Key*/
   /* Pressed User Characteristic Descriptor on the specified Key Fob   */
   /* Service instance.  The first parameter is the Bluetooth Stack ID  */
   /* of the Bluetooth Device.  The second parameter is the InstanceID  */
   /* returned from a successful call to KFS_Initialize_Server().  The  */
   /* final parameter is a pointer to a structure to return the current */
   /* User Description for the specified KFS Service Instance.  The User*/
   /* Description Length should be at least                             */
   /* (KFS_MAXIMUM_SUPPORTED_STRING+1) to hold the Maximum allowable    */
   /* string (plus a single character to hold the NULL terminator) This */
   /* function returns a zero if successful or a negative return error  */
   /* code if an error occurs.                                          */
BTPSAPI_DECLARATION int BTPSAPI KFS_Query_Key_Press_User_Description(unsigned int BluetoothStackID, unsigned int InstanceID, char *UserDescription);
                                                                                                
#ifdef INCLUDE_BLUETOOTH_API_PROTOTYPES                                                         
   typedef int (BTPSAPI *PFN_KFS_Query_Key_Press_User_Description_t)(unsigned int BluetoothStackID, unsigned int InstanceID, char *UserDescription);
#endif
  
   /* The following function is responsible for responding to a KFS Read*/
   /* Client Configuration Request.  The first parameter is the         */
   /* Bluetooth Stack ID of the Bluetooth Device.  The second parameter */
   /* is the InstanceID returned from a successful call to              */
   /* HTS_Initialize_Server().  The third is the Transaction ID of the  */
   /* request.  The final parameter contains the Client Configuration to*/
   /* send to the remote device.  This function returns a zero if       */
   /* successful or a negative return error code if an error occurs.    */
BTPSAPI_DECLARATION int BTPSAPI KFS_Read_Client_Configuration_Response(unsigned int BluetoothStackID, unsigned int InstanceID, unsigned int TransactionID, Boolean_t NotificationsEnabled);

#ifdef INCLUDE_BLUETOOTH_API_PROTOTYPES
   typedef int (BTPSAPI *PFN_KFS_Read_Client_Configuration_Response_t)(unsigned int BluetoothStackID, unsigned int InstanceID, unsigned int TransactionID, Boolean_t NotificationsEnabled);
#endif

   /* The following function is responsible for sending a Key Press     */
   /* notification to a specified remote device.  The first parameter is*/
   /* the Bluetooth Stack ID of the Bluetooth Device.  The second       */
   /* parameter is the InstanceID returned from a successful call to    */
   /* KFS_Initialize_Server().  The third parameter is the ConnectionID */
   /* of the remote device to send the notification to.  The final      */
   /* parameter is the Key Press State to notify.  This function returns*/
   /* a zero if successful or a negative return error code if an error  */
   /* occurs.                                                           */
BTPSAPI_DECLARATION int BTPSAPI KFS_Notify_Key_Press_State(unsigned int BluetoothStackID, unsigned int InstanceID, unsigned int ConnectionID, KFS_Key_Pressed_State_t *KeyPressState);

#ifdef INCLUDE_BLUETOOTH_API_PROTOTYPES
   typedef int (BTPSAPI *PFN_KFS_Notify_Key_Press_State_t)(unsigned int BluetoothStackID, unsigned int InstanceID, unsigned int ConnectionID, KFS_Key_Pressed_State_t *KeyPressState);
#endif

   /* KFS Client API.                                                   */

   /* The following function is responsible for parsing a Key Press     */
   /* State notification received from a remote KFS Server.  The first  */
   /* parameter is the length of the value returned by the remote KFS   */
   /* Server.  The second parameter is a pointer to the data returned by*/
   /* the remote KFS Server.  The final parameter is a pointer to store */
   /* the parsed Key Pressed State value.  This function returns a zero */
   /* if successful or a negative return error code if an error occurs. */
BTPSAPI_DECLARATION int BTPSAPI KFS_Decode_Key_Press_Notification(unsigned int ValueLength, Byte_t *Value, KFS_Key_Pressed_State_t *KeyPressState);

#ifdef INCLUDE_BLUETOOTH_API_PROTOTYPES
   typedef int (BTPSAPI *PFN_KFS_Decode_Key_Press_Notification_t)(unsigned int ValueLength, Byte_t *Value, KFS_Key_Pressed_State_t *KeyPressState);
#endif

#endif
