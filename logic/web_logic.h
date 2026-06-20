#ifndef __WEB_LOGIC_H__
#define __WEB_LOGIC_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the Web Server and Wi-Fi AP
 * 
 * Sets up a SoftAP with SSID "OsmoRemote" and Password "12345678"
 * Starts an HTTP server on port 80 to host the dashboard and API endpoints.
 */
void web_logic_init(void);

#ifdef __cplusplus
}
#endif

#endif // __WEB_LOGIC_H__
