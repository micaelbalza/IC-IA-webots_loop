#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define MAX_LINE_LENGTH 50

int read_txt(int m_displacement, double* local_objective_x, double* local_objective_y, const char* destination_folder) {
    char arq[512] = ""; // Increased buffer size to 512
    char search_file[50];

    char line[MAX_LINE_LENGTH];
        
    // Copy the content of destination_folder to folder_name
    strcpy(arq, destination_folder);
    
    // Concatenate "/response/" to folder_name
    strcat(arq, "response/");
    
    sprintf(search_file, "%d", m_displacement); // convert number to string
    strcat(search_file, ".txt"); // e.g., 0.txt
    
    // Concatenate search_file to folder_name
    strcat(arq, search_file);
    
    printf(" arq - %s",arq);
    
    FILE *response = fopen(arq, "r");

    if (response == NULL) {
        printf("Error: could not open response %s", arq);
        return 1;
    }
    
    while (fgets(line, MAX_LINE_LENGTH, response)) {
        // No need to print the line here, you can remove this line
    }

    // Use a temporary buffer for strtok to avoid modifying the original 'line'
    char temp_buffer[MAX_LINE_LENGTH];
    strcpy(temp_buffer, line);

    char *splited_response1 = strtok(temp_buffer, " ");
    double x = strtod(splited_response1, NULL);

    char *splited_response2 = strtok(NULL, " ");
    double y = strtod(splited_response2, NULL);

    *local_objective_x = x;
    *local_objective_y = y;

    printf("\n Objetivo - x - %lf \n", x); 
    printf("Objetivo - y - %lf \n", y);
    
    fclose(response);
    return 0;
}
