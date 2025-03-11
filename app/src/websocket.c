#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <microhttpd.h>

#define PORT 80

int handle_request(void *cls, struct MHD_Connection *connection, 
                   const char *url, const char *method, 
                   const char *version, const char *upload_data, 
                   size_t *upload_data_size, void **con_cls) 
{
    const char *response_text;

    if (strcmp(url, "/forward") == 0) {
        response_text = "Moving Forward";
        printf("Command Received: Moving Forward\n");
    } else {
        response_text = "Invalid Command";
    }

    struct MHD_Response *response = MHD_create_response_from_buffer(
        strlen(response_text), (void *)response_text, MHD_RESPMEM_PERSISTENT);
    
    int ret = MHD_queue_response(connection, MHD_HTTP_OK, response);
    MHD_destroy_response(response);

    return ret;
}

int main() {
    struct MHD_Daemon *server;
    
    server = MHD_start_daemon(MHD_USE_SELECT_INTERNALLY, PORT, NULL, NULL, 
                              &handle_request, NULL, MHD_OPTION_END);
    if (!server) {
        printf("Failed to start server\n");
        return 1;
    }

    printf("Server running on port %d...\n", PORT);
    getchar(); // Chờ nhấn Enter để thoát

    MHD_stop_daemon(server);
    return 0;
}
