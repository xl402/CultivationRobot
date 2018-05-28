#include <iostream>
using namespace std;
#include <robot_instr.h>
#include <robot_link.h>
#include <robot_delay.h>
#include <vector>
#include <math.h>
#include <stopwatch.h>
#include <numeric>
#include <fstream>
#include <iterator>
#include <algorithm>
#define ROBOT_NUM 8
//#define DEBUG_MOTOR
#define DEBUG_PLANT

#define RUN_ALL
//#define RUN_PLANT_DETECT
//#define RUN_PLANT_COLLECT
//#define RUN_TEST
robot_link rlink;   
  
class robot {
	private:
	int plant_size, cabbage_counter, cauliflower_counter;
	float mean;
	
	bool set_up_test(){	
		int val;     // data from microprocessor
		#ifdef __arm__
		   if (!rlink.initialise ("127.0.0.1")) {  
			   // setup for local hardware
			   cout << "Cannot initialise in mbed" << endl;
			   rlink.print_errs("  ");
			   return 0;
		}
		#else
		   if (!rlink.initialise (ROBOT_NUM)) { // setup the link
			   cout << "Cannot initialise in link" << endl;
			   rlink.print_errs("  ");
			   return 0;
		}   
		#endif
		val = rlink.request (TEST_INSTRUCTION); // send test instruction
		if (val == TEST_INSTRUCTION_RESULT) {   // check result
		  cout << "Test passed" << endl;
		  return 1;                            // all OK, finish
		}
		else if (val == REQUEST_ERROR) {
		  cout << "Fatal errors on link:" << endl;
		  rlink.print_errs();
		}
		else
		  cout << "Test failed (bad value returned)" << endl;
		return 0;                          // error, finish
	}
	
	
	void Actuator_up(){
		int port_0 = get_input(READ_PORT_0);
		rlink.command(WRITE_PORT_0, 0x40 bitor port_0);
		rlink.command(WRITE_PORT_0, 0x80 bitor get_input(READ_PORT_1));
		}

	void Plant_Pickup_v2(){
		rlink.command (MOTOR_1_GO , 0);
		rlink.command (MOTOR_2_GO , 0);
		switch (plant2pick){
			case 1:
				while((get_input(READ_PORT_1) & 1) == 0){
					rlink.command(MOTOR_3_GO, 80);
					}
				cabbage_counter ++;
				rlink.command(MOTOR_3_GO, 0);
				cout <<cabbage_counter<<" "<<cauliflower_counter<<endl;
				break;
			case 2:
				while((get_input(READ_PORT_1) & 2) == 0){
					rlink.command(MOTOR_3_GO, 80+128);
					}
				rlink.command(MOTOR_3_GO, 0);
				cauliflower_counter ++;
				cout <<cabbage_counter<<" "<<cauliflower_counter<<endl;
				break;
		}
		int port_0 = get_input(READ_PORT_0);
		
		if ((plant2pick==1 && cabbage_counter<3) or 
		(plant2pick==2 && cauliflower_counter<3))	{
			rlink.command(WRITE_PORT_0, (0xBF bitand port_0));
			delay(1000);
			port_0 = get_input(READ_PORT_0);
			rlink.command(WRITE_PORT_0, 0x40 bitor port_0);
			delay(1000);
			forward_for_duration(1000,50,-5,true);
			cout<<"Entered"<<endl;
			destroy();	
		}
	}

	void plant_forward_v2(int maxspeed=50, int offset=-5
	, bool reverse = false){
			while((get_input(READ_PORT_0) & 32) == 32 
			and get_input(READ_PORT_0)%8 != 7){
			motor_control(maxspeed, offset, reverse);
			}
			while((get_input(READ_PORT_0) & 32) == 0){
				plant_array.push_back(get_input(ADC4));
				motor_control(maxspeed, offset, reverse);
			}
			analyse_plant();	
			if (plant2pick!=0 && pickup_mode){
				while(get_input(ADC1)<=100){
					motor_control(maxspeed, offset, reverse);
				}
				forward_for_duration(1500,maxspeed,offset,reverse);
				Plant_Pickup_v2();		
			}
							
	}	
	
	void analyse_plant(){
		mean = accumulate(plant_array.begin(), plant_array.end(), 
		0.0/ plant_array.size());
		plant_size = plant_array.size();
		mean = mean/plant_size;
		int port1 = get_input(READ_PORT_1);
		if (plant_size<10){
			destroy();
			cout << "Noise" << endl;
			return;
		}
		else if (plant_size <= SizeThreshold and mean > TypeThreshold ){
			cout << "undersized cabbage" << endl;
			rlink.command(WRITE_PORT_1, (port1 bitand 0x8F) bitor 0x40);
		}
		else if (plant_size <= SizeThreshold and mean <= TypeThreshold ){
			cout << "undersized cauliflower" << endl;
			rlink.command(WRITE_PORT_1, (port1 bitand 0x8F) bitor 0x40);
		}
		else if (plant_size > SizeThreshold and mean <= TypeThreshold ){
			cout << "cauliflower" << endl;
			plant2pick = 2;
			rlink.command(WRITE_PORT_1, (port1 bitand 0x8F) bitor 0x20);
		}
		else if (plant_size > SizeThreshold and mean > TypeThreshold ){
			cout << "cabbage" << endl;
			plant2pick = 1;
			rlink.command(WRITE_PORT_1, (port1 bitand 0x8F) bitor 0x10);
		}
		#ifdef DEBUG_PLANT
		cout<<mean<<"	"<<plant_size<<endl;
		#endif
		mean=0;
		plant_array.clear();
		plant_size = 0;
		
	}	
	
	public:
	vector<int> plant_array;	
	int SizeThreshold, TypeThreshold;
	int plant2pick;//1 for cabbage, 2 for cauliflower
	bool pickup_mode;
	
	/*Initialise the robot with three threshold parameters
	 * namely: number of "interesting" pixels required for the program
	 * to be sure there is a plant in front of it*/
	robot(int th1, int th2){
		set_up_test();
		Actuator_up();
		SizeThreshold = th1;
		TypeThreshold = th2;
		plant2pick = 0;
		pickup_mode = true;
		destroy();
		cabbage_counter = 0;
		cauliflower_counter = 0;
	}
	
	void destroy(){
		mean=0;
		plant_array.clear();
		plant_size = 0;
		plant2pick = 0;
	}
	
	int get_input(request_instruction port_name){
		int data = rlink.request (port_name);
		if (data == -1){ 		
			cout<<"Error: get_input: Null data detected"<<endl;
			}
		return data;
	}
	
	void stop(){
		rlink.command (MOTOR_1_GO , 0);
		rlink.command (MOTOR_2_GO , 0);
		rlink.command (MOTOR_3_GO , 0);
	}
	
	void motor_control(int maxspeed=0, int offset=0
	, bool reverse = false){
		
		int LMotor_speed = 0;
		int RMotor_speed = 0;
		if (reverse) {
			switch (get_input(READ_PORT_0)%8){
			case 0:
				//Robot lost path-recovery
				LMotor_speed = -offset;
				RMotor_speed = 0;
				#ifdef DEBUG_MOTOR
				cout << "ROBOT LOST"<<endl;
				#endif
				break;
			case 4:
				//Sharp turn right
				LMotor_speed = maxspeed;
				RMotor_speed = 0;
				#ifdef DEBUG_MOTOR
				cout << "Sharp right"<<endl;
				#endif
				break;
			case 2:
				//On track, continue straight
				LMotor_speed = maxspeed;
				RMotor_speed = maxspeed;
				break;
			case 6:
				//Slight turn right
				LMotor_speed = maxspeed;
				RMotor_speed = maxspeed*0.1;
				#ifdef DEBUG_MOTOR
				cout << "Bear right"<<endl;
				#endif
				break;
			case 1:
				//Sharp turn left
				LMotor_speed = 0;
				RMotor_speed = maxspeed;
				#ifdef DEBUG_MOTOR
				cout << "Sharp left"<<endl;
				#endif
				break;
			case 5:
				//Unused
				LMotor_speed = maxspeed;
				RMotor_speed = maxspeed;
				#ifdef DEBUG_MOTOR
				cout << "45"<<endl;
				#endif
				break;
			case 3:
				//Slight turn left
				LMotor_speed = maxspeed*0.1;
				RMotor_speed = maxspeed;
				#ifdef DEBUG_MOTOR
				cout << "Bear left"<<endl;
				#endif
				break;
			case 7:
				LMotor_speed = maxspeed;
				RMotor_speed = maxspeed;
				break;
			}
			LMotor_speed += 128;
			RMotor_speed += 128;
		}
		else {
			switch (get_input(READ_PORT_0)%8){
			case 0:
				//Robot lost path-recovery
				LMotor_speed = -offset;
				RMotor_speed = 0;
				#ifdef DEBUG_MOTOR
				cout << "ROBOT LOST"<<endl;
				#endif
				break;
			case 1:
				//Sharp turn right
				LMotor_speed = maxspeed;
				RMotor_speed = 0;
				#ifdef DEBUG_MOTOR
				cout << "Sharp right"<<endl;
				#endif
				break;
			case 2:
				//On track, continue straight
				LMotor_speed = maxspeed;
				RMotor_speed = maxspeed;
				break;
			case 3:
				//Slight turn right
				LMotor_speed = maxspeed;
				RMotor_speed = maxspeed*0.1;
				#ifdef DEBUG_MOTOR
				cout << "Bear right"<<endl;
				#endif
				break;
			case 4:
				//Sharp turn left
				LMotor_speed = 0;
				RMotor_speed = maxspeed;
				#ifdef DEBUG_MOTOR
				cout << "Sharp left"<<endl;
				#endif
				break;
			case 5:
				//Unused
				LMotor_speed = maxspeed;
				RMotor_speed = maxspeed;
				#ifdef DEBUG_MOTOR
				cout << "45"<<endl;
				#endif
				break;
			case 6:
				//Slight turn left
				LMotor_speed = maxspeed*0.1;
				RMotor_speed = maxspeed;
				#ifdef DEBUG_MOTOR
				cout << "Bear left"<<endl;
				#endif
				break;
			case 7:
				LMotor_speed = maxspeed;
				RMotor_speed = maxspeed;
				break;
			}
		}
		rlink.command (MOTOR_1_GO , LMotor_speed+offset);
		rlink.command (MOTOR_2_GO , RMotor_speed);
		}
	
	
	void move_forward_till_node(int maxspeed=0, int offset=0
	, bool reverse = false){
		while(get_input(READ_PORT_0)%8 != 7){
			motor_control(maxspeed, offset, reverse);
		}
		
		#ifdef DEBUG_MOTOR
		cout << "Node reached"<<endl;
		#endif
	}
	
	void connect_nodes(int maxspeed=0, int offset=0,
	bool reverse = false){
		
		while(get_input(READ_PORT_0)%8 != 2){
			motor_control(maxspeed, offset, reverse);
		}				
		#ifdef DEBUG_MOTOR
		cout << "Next straight line reached" <<endl;
		#endif
		
	}
		
	void turn_right(int maxspeed=0, int offset=0
	, bool reverse = false){
		while(get_input(READ_PORT_0)%8 != 1){
		rlink.command (MOTOR_1_GO , maxspeed);
		rlink.command (MOTOR_2_GO , 0);
		}
		#ifdef DEBUG_MOTOR
		cout << "Right turn complete" <<endl;
		#endif
	}
	
	void sharp_turn_right(int maxspeed=0, int offset=0
	, bool reverse = false){
		while(get_input(READ_PORT_0)%8 != 1){
		rlink.command (MOTOR_1_GO , maxspeed);
		rlink.command (MOTOR_2_GO , 0);
		}
		while(get_input(READ_PORT_0)%8 == 1){
		rlink.command (MOTOR_1_GO , maxspeed);
		rlink.command (MOTOR_2_GO , 0);
		}
		while(get_input(READ_PORT_0)%8 != 1){
		rlink.command (MOTOR_1_GO , maxspeed);
		rlink.command (MOTOR_2_GO , 0);
		}			
	
	#ifdef DEBUG_MOTOR
	cout << "Sharp right turn completed" <<endl;
	#endif
			
	}
	
	void turn_45_right(int maxspeed=0, int offset=0
	, bool reverse = false){
		while(get_input(READ_PORT_0)%8 != 3){
		rlink.command (MOTOR_1_GO , maxspeed);
		rlink.command (MOTOR_2_GO , 0);
		}
		while(get_input(READ_PORT_0)%8 == 1){
		rlink.command (MOTOR_1_GO , maxspeed);
		rlink.command (MOTOR_2_GO , 0);
		}			
	
	#ifdef DEBUG_MOTOR
	cout << "45 right turn completed" <<endl;
	#endif
			
	}
	
	void turn_left(int maxspeed=0, int offset=0, bool reverse = false){
		
		while(get_input(READ_PORT_0)%8 != 4){
		rlink.command (MOTOR_1_GO , 0);
		rlink.command (MOTOR_2_GO , maxspeed);
		}	
		#ifdef DEBUG_MOTOR
		cout << "Left turn complete" <<endl;
		#endif
		
	}
	
	void sharp_turn_left(int maxspeed=0, int offset=0
	, bool reverse = false){
		
		while(get_input(READ_PORT_0)%8 != 4){
	
		rlink.command (MOTOR_1_GO , 0);
		rlink.command (MOTOR_2_GO , maxspeed);
		}
		
		while(get_input(READ_PORT_0)%8 == 4){
	
		rlink.command (MOTOR_1_GO , 0);
		rlink.command (MOTOR_2_GO , maxspeed);
		}	
		
		while(get_input(READ_PORT_0)%8 != 4){
	
		rlink.command (MOTOR_1_GO , 0);
		rlink.command (MOTOR_2_GO , maxspeed);
		}	

	#ifdef DEBUG_MOTOR
	cout << "Sharp left turn completed" <<endl;
	#endif		
			
	}
	
	void forward_till_45(int maxspeed=100, int offset=0
	, bool reverse = false){
		while(get_input(READ_PORT_0)%8 != 5){
			motor_control(maxspeed, offset, reverse);
		}
		
		#ifdef DEBUG_MOTOR
		cout << "45 Node reached"<<endl;
		#endif
	}
	
	void forward_for_duration(int time, int maxspeed=100, int offset=0,
	bool reverse = false){
		stopwatch watch;
		watch.start();
		int etime = watch.read();
		while(etime<time){
			motor_control(maxspeed, offset, reverse);
			etime = watch.read();
		}
	}
	
	void turn_180(int time,int maxspeed=100, int offset=0
	, bool reverse = false){
		forward_for_duration(time, maxspeed, offset, reverse);
		while(get_input(READ_PORT_0)%8!= 4){
			rlink.command (MOTOR_1_GO , maxspeed+100);
			rlink.command (MOTOR_2_GO , maxspeed);
		}
		while(get_input(READ_PORT_0)%8!= 1){
			rlink.command (MOTOR_1_GO , maxspeed+100);
			rlink.command (MOTOR_2_GO , maxspeed);	
		}
		while(get_input(READ_PORT_0)%8 != 4){
			rlink.command (MOTOR_1_GO , maxspeed+100);
			rlink.command (MOTOR_2_GO , maxspeed);
		}
		
		#ifdef DEBUG_MOTOR
		cout << "180 turn completed"<<endl;
		#endif
	}
	
	
	void plant_loop(int maxspeed=50, int offset=-5
	, bool reverse = false){
			while(get_input(READ_PORT_0)%8 != 7){			
				plant_forward_v2(maxspeed, offset, reverse);
			}
			destroy();			 
	}	
	
	void ledtest() {
	rlink.command(WRITE_PORT_1, 64+32+16);
	}
	
	
	void docking(int forward_time, int turn_time, int reverse_time,
	 bool isCauli, int maxspeed = 100, int offset = -5){
		connect_nodes(maxspeed, offset);
		forward_for_duration(forward_time,maxspeed, offset);
		stop();
		delay(500);
		stopwatch watch;
		watch.start();
		int etime = watch.read();
		while(etime<turn_time){
			rlink.command (MOTOR_1_GO , maxspeed+128);
			rlink.command (MOTOR_2_GO , maxspeed);
			etime = watch.read();
		}
		watch.stop();
		watch.start();
		etime = watch.read();
		while(etime<reverse_time){
			rlink.command (MOTOR_1_GO , maxspeed+128);
			rlink.command (MOTOR_2_GO , maxspeed+128);
			etime = watch.read();
		}
		stop();
		delay(500);
		if (isCauli){
			while((get_input(READ_PORT_1) & 2) == 0){
					rlink.command(MOTOR_3_GO, 80+128);
			}
		}
		else{
			while((get_input(READ_PORT_1) & 1) == 0){
					rlink.command(MOTOR_3_GO, 80);
			}
		}
		stop();
		delay(500);
		rlink.command(WRITE_PORT_0, 0x7F bitand get_input(READ_PORT_0));
		delay(500);
		rlink.command(WRITE_PORT_0, 0x80 bitor get_input(READ_PORT_0));
		turn_left(maxspeed, offset);
		delay(1000);
		rlink.command(WRITE_PORT_0, 0x40 bitor get_input(READ_PORT_0));
	}
	
	void eject_test(){
		while((get_input(READ_PORT_1) & 1) == 0){
					rlink.command(MOTOR_3_GO, 80);
			}
		delay(1000);
		rlink.command(WRITE_PORT_0, 0x7F bitand get_input(READ_PORT_0));
		delay(1000);
		rlink.command(WRITE_PORT_0, 0x80 bitor get_input(READ_PORT_0));
		while((get_input(READ_PORT_1) & 2) == 0){
					rlink.command(MOTOR_3_GO, 80+128);
			}
		delay(1000);
		rlink.command(WRITE_PORT_0, 0x7F bitand get_input(READ_PORT_0));
		delay(1000);
		rlink.command(WRITE_PORT_0, 0x80 bitor get_input(READ_PORT_0));
		
	}
	
	void actuator_test(){
		int port_0 = get_input(READ_PORT_0);
		rlink.command(WRITE_PORT_0, (0xBF bitand port_0));
		delay(1000);
		rlink.command(WRITE_PORT_0, 0x40 bitor port_0);
	}
	
	void turn_table_test(){
		while((get_input(READ_PORT_1) & 1) == 0){
					rlink.command(MOTOR_3_GO, 80);
			}
		delay(1000);
		while((get_input(READ_PORT_1) & 2) == 0){
					rlink.command(MOTOR_3_GO, 80+128);
			}
		delay(1000);
	}
	void cout_distance(){
		while(true){
			cout<<get_input(ADC1)<<endl;
		}
	}
	
	void lift_arm(){
		int port_0 = get_input(READ_PORT_0);
		rlink.command(WRITE_PORT_0, 0x40 bitor port_0);
	}
	
};

int main ()
{
	
	robot rob(35, 40);
	rob.lift_arm();
	
	#ifdef RUN_TEST
	delay(5000);
	rob.turn_table_test();
	rob.actuator_test();
	rob.eject_test();
	#endif
	
	#ifdef RUN_PLANT_DETECT
	rob.pickup_mode = false;
	rob.plant_loop(50,-5);
	#endif
	
	#ifdef RUN_PLANT_COLLECT
	rob.pickup_mode = true;
	rob.plant_loop(50,-5);
	#endif
	
	#ifdef RUN_ALL
	rob.lift_arm();
	delay(10000);
	//MAIN CODE! DO NOT DELETE
	rob.move_forward_till_node(100, -5);
	rob.connect_nodes(100, -5);
	rob.move_forward_till_node(100, -5);
	rob.connect_nodes(100, -5);
	rob.move_forward_till_node(100, -5);
	rob.turn_180(1300, 100, -5);
	rob.move_forward_till_node(100, -5,true);
	rob.connect_nodes(100,-5);	
	rob.pickup_mode = true;
	rob.plant_loop(50,-5);
	rob.turn_left(100,-5);
	//Return sequence
	rob.move_forward_till_node(100,-5);
	rob.turn_right(100,-5);
	rob.move_forward_till_node(100,-5);
	rob.docking(4000,2400,1000,true,100,-5);
	rob.move_forward_till_node(100,-5);
	rob.connect_nodes(100,-5);
	rob.move_forward_till_node(100,-5);
	rob.turn_left(100,-5);
	rob.move_forward_till_node(100,-5);
	rob.turn_180(500, 100,-5);
	rob.move_forward_till_node(100,-5);
	rob.docking(3000,2400,1000,false,100,-5);
	rob.turn_left(100,-5);
	rob.move_forward_till_node(100,-5);
	rob.turn_left(100,-5);
	rob.move_forward_till_node(100,-5);
	rob.turn_left(100,-5);
	rob.move_forward_till_node(100,-5);
	rob.turn_right(100,-5);
	rob.move_forward_till_node(100,-5);
	rob.connect_nodes(100,-5);
	rob.move_forward_till_node(100,-5);
	rob.stop();
	#endif
}

