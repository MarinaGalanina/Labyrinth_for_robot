

#include <stdio.h>
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <math.h>
#include <webots/position_sensor.h>
#include <webots/keyboard.h>
#include <stdlib.h>

#define TIME_STEP 32
#define PLYTKA 0.12 //PLYTKA length
#define OS 0.0568 //OS length 0.0568
#define WHEEL 0.02002 //wheel radius 0.02002
#define SPEED 2

// binarne 00000001, 00000010...
#define  ZACHOD    1   
#define  POLUDNIE   2    
#define  WCHOD    4    
#define  POLNOC   8    

#define WIERSZY 16
#define KOLUMNY 16
#define ROZMIAR WIERSZY * KOLUMNY 
#define ODWIEDZONE 64
// 1- klawiszy, 2- znajdz najkrotsza sciezke, 3 - przejscie ta sciezka
#define MODE 2 

//zadanie celu dla trybow
#if MODE == 3
  #define CELL 136
#elif MODE == 2
  #define CELL 1
#else 
  #define CELL 0
#endif

//mapa
void dodaj_sciane(unsigned char map[],unsigned char position, char orientation, unsigned char Wall);              
void stworz_map(unsigned char map[]);
void stworz_cel(unsigned char dist[], unsigned char target);
              
//algorytmy                                         
char zmien_orientacje(char orientation, char action);
unsigned char zmien_pozyje(unsigned char position, char orientation);
void floodfill(unsigned char map[], unsigned char current_position, unsigned char distance[]);  
unsigned char gdzie_jechac(unsigned char map[], unsigned char current_position, unsigned char distance[], unsigned char orientation);
unsigned char zmien_cel( unsigned char map[], unsigned char position, unsigned char target, unsigned char distance[]);
 
//ruch             
void na_1_plytke(WbDeviceTag left_motor, WbDeviceTag right_motor, WbDeviceTag ps_left, WbDeviceTag ps_right);
void obrot(char key, WbDeviceTag left_motor, WbDeviceTag right_motor, WbDeviceTag ps_left, WbDeviceTag ps_right);
void wait_move_end(WbDeviceTag ps_left, WbDeviceTag ps_right);
void korekcja_predkosci(double left_wall, double right_wall, WbDeviceTag left_motor, WbDeviceTag right_motor);
              
void print_array(unsigned char map[],int action);

int main(int argc, char **argv) {
  wb_robot_init();
  
  unsigned char map[ROZMIAR] = 
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  
  unsigned char distance[ROZMIAR];
  
  unsigned char position = 0;
  unsigned char direction = 8; 
  unsigned char target = CELL;
  char key;
  char mode = MODE; 
  int open = 1; 
  stworz_map(map);

  char orientation = POLNOC; 
  
  // 8 distance sensorow
  WbDeviceTag ps[8]; 
  char ps_names[8][4] = {
  "ps0", "ps1", "ps2", "ps3",
  "ps4", "ps5", "ps6", "ps7"
  };
  
  for (int i = 0; i < 8 ; i++)
  {
    ps[i] = wb_robot_get_device(ps_names[i]);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  
  WbDeviceTag ps_left = wb_robot_get_device("left wheel sensor");
  WbDeviceTag ps_right = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(ps_left, TIME_STEP);
  wb_position_sensor_enable(ps_right, TIME_STEP);
  
  //klawisz 
  wb_keyboard_enable(TIME_STEP);
  wb_motor_set_velocity(left_motor,SPEED);
  wb_motor_set_velocity(right_motor,SPEED);
     
  while (wb_robot_step(TIME_STEP) != -1) {
           
   double avg_2 = 0; //prawo
   double avg_4 = 0; //tył
   double avg_5 = 0; //lewo
   double avg_7 = 0; //przód
   double n = 5;
 
   double ps_values[8];
    
   for(int i = 0; i < 4; i++) 
   {
     for(int i = 0; i<8; i++)
     {
       ps_values[i] = wb_distance_sensor_get_value(ps[i]); 
     }    
     avg_2 += ps_values[2];

     avg_4 += ps_values[4];

     avg_5 += ps_values[5];

     avg_7 += ps_values[7];
        
     wb_robot_step(TIME_STEP); 
   }
   
   avg_2 = avg_2 / n;
   avg_4 = avg_4 / n;
   avg_5 = avg_5 / n;
   avg_7 = avg_7 / n;   
       
     bool left_wall = avg_5 > 80.0;
     bool front_wall = avg_7 > 80.0;
     bool right_wall = avg_2 > 80.0;
     bool back_wall = avg_4 > 80.0;
     
     if(left_wall)  dodaj_sciane(map, position, orientation, ZACHOD);
     if(front_wall) dodaj_sciane(map, position, orientation, POLNOC);
     if(right_wall) dodaj_sciane(map, position, orientation, WCHOD);
     if(back_wall)  dodaj_sciane(map, position, orientation, POLUDNIE);
     
  //kontrola za pomocą klawisza       
  if(mode == 1)
  {
    
    switch(key = wb_keyboard_get_key())
    {
      case 'W':   //góra
        na_1_plytke(left_motor,right_motor, ps_left, ps_right);
        break;
            
      case 'D': //prawo
      case 'A': //lewo
        obrot(key, left_motor, right_motor, ps_left, ps_right);
        break;
      case 'S':
        break;
    }
  
     
    }
   if(mode == 2)
   {
 //floodfill 
        
  //mapa
  printf("MAPA \n"); 
  print_array(map, 0); 
  printf("MAPA \n"); 
  
  stworz_cel(distance, target); 
  floodfill(map, 0, distance); 
  direction = gdzie_jechac(map, position, distance, orientation);
  
  
  if(orientation == direction) 
  {
    if(left_wall && right_wall) korekcja_predkosci(avg_5, avg_2, left_motor, right_motor);
    na_1_plytke(left_motor,right_motor, ps_left, ps_right);
  }
  else if( !((orientation == ZACHOD) && (direction == POLNOC)) != !( (orientation / 2) == direction) ) 
  {
   orientation = zmien_orientacje(orientation, 'D');
   obrot('D', left_motor, right_motor, ps_left, ps_right);
   na_1_plytke(left_motor,right_motor, ps_left, ps_right);
  }
  else if( !((orientation == POLNOC) && (direction == ZACHOD)) != !( (orientation * 2) == direction) ) 
  {
    orientation = zmien_orientacje(orientation, 'A');
    obrot('A', left_motor, right_motor, ps_left, ps_right);
    na_1_plytke(left_motor,right_motor, ps_left, ps_right);
  }
  else if( !( (orientation * 4) == direction) != !( (orientation / 4) == direction) ) 
  {
    orientation = zmien_orientacje(orientation, 'S');
    obrot('S', left_motor, right_motor, ps_left, ps_right);
    na_1_plytke(left_motor,right_motor, ps_left, ps_right);
  }
   
             
   position = zmien_pozyje(position, orientation); 
   map[position] = map[position] | ODWIEDZONE; //ODWIEDZONE PLYTKA
   
   if(position == target)
   {
     target = zmien_cel(map, position, target, distance);
   }
   
 
   }
   else if(mode == 3)
   {
            //szybka juz zrobiona scierzka
     if(open)
     {
       FILE *trasa;
       trasa = fopen("trasa.txt","rb");
       if(trasa == NULL)
       {
         printf("ERROR TRASA NULL\n");
         exit(1);
       }
       fread(distance,sizeof(unsigned char),ROZMIAR,trasa);
       fclose(trasa);
       print_array(distance, 0);
       
       FILE *mapa;
       mapa = fopen("mapa.txt","rb");
       if(mapa == NULL)
       {
         printf("ERROR MAPA NULL\n");
         exit(1);
       }
       fread(map,sizeof(unsigned char),ROZMIAR,mapa);
       fclose(mapa);
       print_array(map, 1);
       
       open = 0;
     }
     
 
       direction = gdzie_jechac(map, position, distance, orientation);
  
  
      if(orientation == direction) 
      {
        if(left_wall && right_wall) korekcja_predkosci(avg_5, avg_2, left_motor, right_motor);
        na_1_plytke(left_motor,right_motor, ps_left, ps_right);
      }
      else if( !((orientation == ZACHOD) && (direction == POLNOC)) != !( (orientation / 2) == direction) ) 
      {
       orientation = zmien_orientacje(orientation, 'D');
       obrot('D', left_motor, right_motor, ps_left, ps_right);
       na_1_plytke(left_motor,right_motor, ps_left, ps_right);
      }
      else if( !((orientation == POLNOC) && (direction == ZACHOD)) != !( (orientation * 2) == direction) ) 
      {
        orientation = zmien_orientacje(orientation, 'A');
        obrot('A', left_motor, right_motor, ps_left, ps_right);
        na_1_plytke(left_motor,right_motor, ps_left, ps_right);
      }
      else if( !( (orientation * 4) == direction) != !( (orientation / 4) == direction) ) 
      {
        orientation = zmien_orientacje(orientation, 'S');
        obrot('S', left_motor, right_motor, ps_left, ps_right);
        na_1_plytke(left_motor,right_motor, ps_left, ps_right);
      }
                 
       position = zmien_pozyje(position, orientation); 
       
       if(position == target)
       {
         printf("koniec\n");
         wb_robot_cleanup();
         exit(0);
       }
           
   }
   
  }

  wb_robot_cleanup();

  return 0;
}


void na_1_plytke(WbDeviceTag left_motor, WbDeviceTag right_motor, WbDeviceTag ps_left, WbDeviceTag ps_right)
{
   double revolution_l, revolution_r, rev;
  
  rev = PLYTKA/WHEEL; 
  revolution_l = wb_position_sensor_get_value(ps_left);
  revolution_r = wb_position_sensor_get_value(ps_right);
  revolution_l += rev;
  revolution_r += rev;
  wb_motor_set_position(left_motor, revolution_l);
  wb_motor_set_position(right_motor, revolution_r);
  printf("prosto \n");
  wait_move_end(ps_left, ps_right);
}


void obrot(char key, WbDeviceTag left_motor, WbDeviceTag right_motor, WbDeviceTag ps_left, WbDeviceTag ps_right)
{
  double revolution_l, revolution_r, rev;
  
  rev = (M_PI/2) * OS/2/WHEEL; 
  
  revolution_l = wb_position_sensor_get_value(ps_left);
  revolution_r = wb_position_sensor_get_value(ps_right);
  
  wb_motor_set_velocity(right_motor,SPEED);
  wb_motor_set_velocity(left_motor,SPEED);
  
  switch(key)
  {
    case 'D':
    revolution_l += rev;
    revolution_r -= rev;
    wb_motor_set_position(left_motor, revolution_l);
    wb_motor_set_position(right_motor, revolution_r);
    printf("prawo \n");
    break;
    case 'A':
    revolution_l -= rev;
    revolution_r += rev;
    wb_motor_set_position(left_motor, revolution_l);
    wb_motor_set_position(right_motor, revolution_r);
    printf("lewo \n");
    break;
    case 'S':
    rev = M_PI * OS/2/WHEEL;
    revolution_l += rev;
    revolution_r -= rev;
    wb_motor_set_position(left_motor, revolution_l);
    wb_motor_set_position(right_motor, revolution_r);
    printf("tyl \n");
    break;
  }
  wait_move_end(ps_left, ps_right);
}

//gdy stop
void wait_move_end(WbDeviceTag ps_left, WbDeviceTag ps_right)
{
double dist_left, dist_left_2, dist_right, dist_right_2;
 
  do
     { 
       dist_left = wb_position_sensor_get_value(ps_left); 
       dist_right = wb_position_sensor_get_value(ps_right);
       wb_robot_step(TIME_STEP);
       dist_left_2 = wb_position_sensor_get_value(ps_left);
       dist_right_2 = wb_position_sensor_get_value(ps_right);
         
     }while((dist_left != dist_left_2) && (dist_right != dist_right_2) );
}

//korekcja
void korekcja_predkosci(double left_wall, double right_wall, WbDeviceTag left_motor, WbDeviceTag right_motor)
{
  if(fabs(left_wall - right_wall) > 20)
       {
         if(left_wall > right_wall) 
         {
           wb_motor_set_velocity(right_motor,SPEED * 0.96);
           wb_motor_set_velocity(left_motor,SPEED);
         }
         else 
         {
           wb_motor_set_velocity(right_motor,SPEED);
           wb_motor_set_velocity(left_motor,SPEED * 0.96);
         }
       }
  else if(fabs(left_wall - right_wall) > 10)
       {
         if(left_wall > right_wall) 
         {
           wb_motor_set_velocity(right_motor,SPEED * 0.98);
           wb_motor_set_velocity(left_motor,SPEED);
         }
         else 
         {
           wb_motor_set_velocity(right_motor,SPEED);
           wb_motor_set_velocity(left_motor,SPEED * 0.98);
         }
       }
}
                                  
void dodaj_sciane(unsigned char map[],unsigned char position, char orientation, unsigned char Wall)
{
  if(orientation == WCHOD)
  {
    if(Wall != ZACHOD) Wall /= 2;
    else Wall = 8;
  }
  else if(orientation == POLUDNIE)
  {
    if( (Wall == ZACHOD) || (Wall == POLUDNIE) ) Wall *= 4;
    else Wall /= 4;
  }
  else if(orientation == ZACHOD)
  {
    if(Wall != POLNOC) Wall *= 2;
    else Wall = 1;
  }
  
  
  map[position] = map[position] | Wall; 
  
  if( Wall == POLNOC ) 
  { 
    position = position + KOLUMNY;     
    map[position] = map[position] | POLUDNIE; 
  } 
  if( Wall == WCHOD ) 
  { 
    position = position + 1; 
    map[position] = map[position] | ZACHOD; 
  } 
  if( Wall == POLUDNIE ) 
  { 
    position = position - KOLUMNY;   
    map[position] = map[position] | POLNOC;  
  } 
  if( Wall == ZACHOD ) 
  { 
  position = position - 1;   
  map[position] = map[position] | WCHOD; 
  } 
}

void stworz_map(unsigned char map[])
{
  map[0] = map[0] | ODWIEDZONE; 
  
  for(int i = 0; i < 16 ;i++)
  {
    map[i] = map[i] | POLUDNIE;
  }
  for(int i = 240; i < 256 ;i++)
  {
   map[i] = map[i] | POLNOC;
  }
  for(int i = 0; i < 241 ;i+=16)
  {
   map[i] = map[i] | ZACHOD;
  }
  for(int i = 15; i < 256 ;i+=16)
  {
   map[i] = map[i] | WCHOD;
  }
  
  print_array(map, 0);
  
}


void stworz_cel(unsigned char dist[], unsigned char target)
{
  for(int i = 0; i < ROZMIAR; i++)
  {
    dist[i] = 255;
  }
  dist[target] = 0;
}

char zmien_orientacje(char orientation, char action)
{
  switch(action)
  {
    case 'D': //prawo
      if(orientation == ZACHOD) orientation = POLNOC; 
      else orientation = orientation / 2;
      break;
    case 'A': //lewo
      if(orientation == POLNOC) orientation = ZACHOD; 
      else orientation = orientation * 2;
      break;
    case 'S': //tył
      if((orientation == POLNOC) || (orientation == WCHOD)) orientation = orientation / 4; 
      else orientation = orientation * 4;
      break;
  }
  printf("orientacja: %d \n", orientation);
  return orientation;
} 
                   

unsigned char zmien_pozyje(unsigned char position, char orientation)
{
  if( orientation == POLNOC ) 
  { 
    position = position + KOLUMNY;          
  } 
  if( orientation == WCHOD ) 
  { 
    position = position + 1; 
  } 
  if( orientation == POLUDNIE ) 
  { 
    position = position - KOLUMNY;    
  } 
  if( orientation == ZACHOD ) 
  { 
    position = position - 1;    
  } 
  
  
   return position;
}

//floodfill 
void floodfill(unsigned char map[], unsigned char current_position, unsigned char distance[])
{
bool search = true;
while(search) 
{
  search = false;
  for(int i = 0; i < ROZMIAR; i++)
  {
    if(distance[i] < 255)
    {
      if((map[i] & POLNOC) != POLNOC)
      {
        
        if(distance[i + KOLUMNY] == 255 || ((distance[i] +1) < distance[i + KOLUMNY])) 
        {
          distance[i + KOLUMNY] = distance[i] + 1; //POLNOC 
          search = true;
        }  
      }
      if((map[i] & WCHOD) != WCHOD)
      {
        if(distance[i + 1] == 255 || ((distance[i] +1) < distance[i + 1])) 
        {
          distance[i + 1] = distance[i] + 1; //WCHOD
          search = true;
        }
      }
      if((map[i] & POLUDNIE) != POLUDNIE)
      {
        if(distance[i - KOLUMNY] == 255 || ((distance[i] +1) < distance[i - KOLUMNY])) 
        {
          distance[i - KOLUMNY] = distance[i] + 1; //POLUDNIE
          search = true;
        }
      }
      if((map[i] & ZACHOD) != ZACHOD)
      {
        if(distance[i - 1] == 255 || ((distance[i] +1) < distance[i - 1]))  
        {
          distance[i - 1] = distance[i] + 1; //ZACHOD
          search = true;
        }
      }
    } 
  }
}
  
printf("\n TRASA \n");
print_array(distance, 0);
printf("\n TRASA \n");

}


unsigned char gdzie_jechac(unsigned char map[], unsigned char current_position, unsigned char distance[], unsigned char orientation)
{
  unsigned char best_neighbor = 255;
  unsigned char direction = POLNOC;
  
      if((map[current_position] & POLNOC) != POLNOC)
      {
        if(distance[current_position + KOLUMNY] <= best_neighbor) 
        {
          if(distance[current_position + KOLUMNY] < best_neighbor)
          {
            best_neighbor = distance[current_position + KOLUMNY];
            direction = POLNOC; 
          }
          else if(orientation == POLNOC) direction = POLNOC;
        }  
      }
      if((map[current_position] & WCHOD) != WCHOD)
      {
        if(distance[current_position + 1] <= best_neighbor) 
        {
          if(distance[current_position + 1] < best_neighbor)
          {
            best_neighbor = distance[current_position + 1];
            direction = WCHOD;
          }
          else if(orientation == WCHOD) direction = WCHOD;
        }
      }
      if((map[current_position] & POLUDNIE) != POLUDNIE)
      {
        if(distance[current_position - KOLUMNY] <= best_neighbor) 
        {
        if(distance[current_position - KOLUMNY] < best_neighbor)
        {
          best_neighbor = distance[current_position - KOLUMNY];
          direction = POLUDNIE;
        }
        else if(orientation == POLUDNIE) direction = POLUDNIE;
        }
      }
      if((map[current_position] & ZACHOD) != ZACHOD)
      {
        if(distance[current_position - 1] <= best_neighbor)  
        {
          if(distance[current_position - 1] < best_neighbor)
          {
          best_neighbor = distance[current_position - 1];
          direction = ZACHOD;
          }
          else if(orientation == ZACHOD) direction = ZACHOD;
        }
      }
      
      return direction;
}


unsigned char zmien_cel( unsigned char map[], unsigned char position, unsigned char target, unsigned char distance[])
{
     bool search = true;
     int i = 0;
     
     while(search) 
     {
       if( !(map[i] & ODWIEDZONE) ) 
       {
         target = i;
         search = false;
         printf("target = %d\n",target);
       }
       else i++;
       if(i == 256) //i ODWIEDZONE-cel 
       {
         target = 136;
         printf("target = %d\n",target);
         search = false;
         i = 0;
       }
       
     }
     if((position == target) && (i == 0) && (target == 136)) //do plikiu
     {
       printf("HURRA KONIEC!!!!!!!!! \n");
       FILE *trasa;
       trasa = fopen("trasa.txt","wb");
       if(trasa == NULL)
       {
         printf("ERROR \n");
         exit(1);
       }
       fwrite(distance,sizeof(unsigned char),ROZMIAR,trasa);
       fclose(trasa);
       trasa = fopen("trasa.txt","rb");
       if(trasa == NULL)
       {
         printf("ERROR \n");
         exit(1);
       }
       fread(distance,sizeof(unsigned char),ROZMIAR,trasa);
       fclose(trasa);
       print_array(distance, 0);
       
       FILE *mapa;
       mapa = fopen("mapa.txt","wb");
       if(mapa == NULL)
       {
         printf("ERROR \n");
         exit(1);
       }
       fwrite(map,sizeof(unsigned char),ROZMIAR,mapa);
       fclose(mapa);
       mapa = fopen("mapa.txt","rb");
       if(mapa == NULL)
       {
         printf("ERROR \n");
         exit(1);
       }
       fread(map,sizeof(unsigned char),ROZMIAR,mapa);
       fclose(mapa);
       print_array(map, 1);
       wb_robot_cleanup();
       exit(0);
     } 
     
   
   
   return target;  
}

                                           
void print_array(unsigned char map[],int action)
{
  printf(" \n");
  if(action == 0) 
  {
    for(int i = 240, k = 1; i >= 0; i++)
     {
       printf("%3d ",map[i]);
       if(k == 16)
       {
         printf("\n");
         i-=32;
         k = 1;
       }
       else k++;
     }
   }
   else if(action == 1) 
   {
     unsigned char array[ROZMIAR];
     
     for(int i = 0; i <= 255; i++) 
     {
       array[i] = map[i];
       array[i] -= 64; 
     }
     for(int i = 240, k = 1; i >= 0; i++)
     {
       printf("%3d ",array[i]);
       if(k == 16)
       {
         printf("\n");
         i-=32;
         k = 1;
       }
       else k++;
     }
   }
   printf(" \n");  
}
              
