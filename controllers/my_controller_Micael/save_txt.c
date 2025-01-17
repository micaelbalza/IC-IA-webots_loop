#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int save_txt(struct pR_node* pR_displacement, struct node* pDP_displacement, struct pO_node* pO_displacement, int m_displacement, int K_points, double final_objective[2], const char* destination_folder)
{
    FILE *arq;
    char Str[100];
    int result;
    int fail = 0;
    int n;

    char complete_file2[512] = ""; // Increased buffer size to 512
    char file[50];

    // Copy the content of destination_folder to folder_name
    strcpy(complete_file2, destination_folder);

    // Concatenate "/response/" to folder_name
    strcat(complete_file2, "request/");
    
    n = sprintf(file, "%d", m_displacement); // convert number in string
    strcat(file, ".txt"); // 0.txt
    
    strcat(complete_file2, file); 

    arq = fopen(complete_file2,"w");
    if (arq == NULL)
    {
      printf("Erro ao criar o arquivo!");
      exit(1);
    }
    
    //strcat(strcpy(Str, "m_displacement - "), snum); // concatenate two strings and save in Str
  
    // -------- cabeçalho ------------------------------------------------------  
    n = 0;
    int m = 0;
    
    // K_points
    n=sprintf (Str, "K_points/-/%d \n", K_points); // convert and concatenate K_points to string [Str]
    m=fputs(Str, arq); // add the line
    if(n<0 || m == EOF) fail = 1;
    
    // m_displacement
    n=sprintf (Str, "m_displacement/-/%d \n", m_displacement); // convert and concatenate to string
    m=fputs(Str, arq); // add the line
    if(n<0 || m == EOF) fail = 1;
  
    // pO.size
    n=sprintf (Str, "pO.size/-/%d \n", pO_displacement->pO_m.size); // convert and concatenate to string
    m=fputs(Str, arq); // add the line
    if(n<0 || m == EOF) fail = 1;
  
    // next line
    strcpy(Str, "\n");
    m=fputs(Str, arq);
    if(m == EOF) fail = 1;
    
  
    // pR_displacement--------------------------------------------------------------
    strcpy(Str, "pR - X/-/Y \n");
    m=fputs(Str, arq);
    if(m == EOF) fail = 1;
  
    n=sprintf (Str, "%10f/-/%10f \n", pR_displacement->pR_m.X,pR_displacement->pR_m.Y); // convert and concatenate K_points to string [Str]
    m=fputs(Str, arq); // add the line
    if(n<0 || m == EOF) fail = 1;
    
    // next line
    strcpy(Str, "\n");
    m=fputs(Str, arq);//---------------------------------------------------------------
    if(m == EOF) fail = 1;
  
  
    // Final objective
    strcpy(Str, "final_objective - X/-/Y \n");
    m=fputs(Str, arq);
    if(m == EOF) fail = 1;
  
    n=sprintf (Str, "%10f/-/%10f \n", final_objective[0] , final_objective[1]); 
    m=fputs(Str, arq); // add the line
    if(n<0 || m == EOF) fail = 1;
  
  
    // next line
    strcpy(Str, "\n");
    m=fputs(Str, arq);
    if(m == EOF) fail = 1;
  
  
  
    // pDP_displacement
    strcpy(Str, "pDP - X/-/Y \n");
    m=fputs(Str, arq);
    for(int i=0; i< K_points; i++){
      n=sprintf (Str, "%10f/-/%10f \n", pDP_displacement->delimiting_polygon.Xdp[i] ,pDP_displacement->delimiting_polygon.Ydp[i]); // convert and concatenate K_points to string [Str]
      m=fputs(Str, arq); // add the line
      if(n<0 || m == EOF) fail = 1;
    }
      // next line
    strcpy(Str, "\n");
    m=fputs(Str, arq);
    if(m == EOF) fail = 1;
  
  
  
  
   // pO_displacement
    strcpy(Str, "pO - X/-/Y \n");
    m=fputs(Str, arq);
    if(m == EOF) fail = 1;
  
    for(int i=0; i< pO_displacement->pO_m.size; i++){
      n=sprintf (Str, "%10f/-/%10f \n", pO_displacement->pO_m.x[i] ,pO_displacement->pO_m.y[i]); // convert and concatenate K_points to string [Str]
      m=fputs(Str, arq); // add the line
      if(n<0 || m == EOF) fail = 1;
    }
      // next line
    strcpy(Str, "\n");
    m=fputs(Str, arq);
    if(m == EOF) fail = 1;
    
    
   
  
    result = fclose(arq);
    if((arq == NULL)||(result != 0)||(fail == 1)){
      fail = 1;
    }else{
      fail = 0;
    }
    return fail;
  }
