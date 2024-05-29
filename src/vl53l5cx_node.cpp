#include <unistd.h>
#include <signal.h>
#include <dlfcn.h>
#include <memory>
#include <fcntl.h>

#include <stdio.h>
#include <string.h>

#include "rclcpp/rclcpp.hpp"

#include "vl53l5cx_api.h"

class RangingSensor : public rclcpp::Node {
    public:
        RangingSensor() : Node("vl53l5cx_node") {

            /*********************************/
            /*   Power on sensor and init    */
            /*********************************/

            /* Initialize channel com */
            status = vl53l5cx_comms_init(&Dev.platform);
            if(status)
            {
                RCUTILS_LOG_ERROR_NAMED(get_name(), "VL53L5CX comms init failed.\n\r");
            }

            status = example1(&Dev);
			printf("\n");
            
            vl53l5cx_comms_close(&Dev.platform);
        };
        ~RangingSensor(){};

        

        int example1(VL53L5CX_Configuration *p_dev)
        {

            /*********************************/
            /*   VL53L5CX ranging variables  */
            /*********************************/

            uint8_t 				status, loop, isAlive, isReady, i;
            VL53L5CX_ResultsData 	Results;		/* Results data from VL53L5CX */


            /*********************************/
            /*   Power on sensor and init    */
            /*********************************/

            /* (Optional) Check if there is a VL53L5CX sensor connected */
            status = vl53l5cx_is_alive(p_dev, &isAlive);
            if(!isAlive || status)
            {
                printf("VL53L5CX not detected at requested address\n");
                return status;
            }

            /* (Mandatory) Init VL53L5CX sensor */
            status = vl53l5cx_init(p_dev);
            if(status)
            {
                printf("VL53L5CX ULD Loading failed\n");
                return status;
            }

            printf("VL53L5CX ULD ready ! (Version : %s)\n",
                    VL53L5CX_API_REVISION);


            /*********************************/
            /*         Ranging loop          */
            /*********************************/

            status = vl53l5cx_start_ranging(p_dev);

            loop = 0;
            while(loop < 10)
            {
                /* Use polling function to know when a new measurement is ready.
                * Another way can be to wait for HW interrupt raised on PIN A3
                * (GPIO 1) when a new measurement is ready */
        
                isReady = wait_for_dataready(&p_dev->platform);

                if(isReady)
                {
                    vl53l5cx_get_ranging_data(p_dev, &Results);

                    /* As the sensor is set in 4x4 mode by default, we have a total 
                    * of 16 zones to print. For this example, only the data of first zone are 
                    * print */
                    printf("Print data no : %3u\n", p_dev->streamcount);
                    for(i = 0; i < 16; i++)
                    {
                        printf("Zone : %3d, Status : %3u, Distance : %4d mm\n",
                            i,
                            Results.target_status[VL53L5CX_NB_TARGET_PER_ZONE*i],
                            Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*i]);
                    }
                    printf("\n");
                    loop++;
                }
            }

            status = vl53l5cx_stop_ranging(p_dev);
            printf("End of ULD demo\n");
            return status;
        }

    private:
        int status;
        VL53L5CX_Configuration Dev;
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RangingSensor>());
    rclcpp::shutdown();
    return 0;
}