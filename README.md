# Traffic-Light-Controller
Designed of a traffic light control system using CCS IDE with C programming language on Arm Cortex M4 Tiva C board as microcontroller. The system contains simply of two traffic lights. One allows cars to move from north to south, the other one allows cars to move from east to west. Beside each traffic, there is a pedestrian traffic light. The pedestrian has to press on a button to have his light green to cross the street safely.

The East to West traffic is supposed to start firstly once the system is working. All RED lights of other traffics will be on at start except the GREEN light of East to West traffic will blink for 5 seconds, then 
turns to the YELLOW for 3 seconds to warn the cars on the EW that the RED light will be on. Red light will be on for 1 second and the green light of the North to South traffic will blink immediately. So, the cars on NS can move, and that will be for 5 seconds as well.
Then, the yellow will turn on for 3 seconds to warn cars on NS way to stop and RED light will be on for a second and the sequence is repeated.
Once a pedestrian needs to pass the road of EW, and presses the button (Switch 1). The RED light of EW will be on immediately and the pedestrian traffic GREEN light will be on for 2 seconds.
It then returns to red again and the car traffic continues the timing from where it stopped. The same sequence goes for a pedestrian needs to pass the road of NS and uses the NS pedestrian traffic button.
