function TestLCM



lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();
visionMsg = cheetahlcm.vision_data_t();

if true
    visionMsg.cpu_calc_time_microseconds = 0;
    visionMsg.enabled = 1;
    visionMsg.N_obstacles = 1;
    visionMsg.d = zeros(10,1);
    visionMsg.h = zeros(10,1);
    visionMsg.d(1) = 0.9;
    visionMsg.h(1) = 0.21;
else
    visionMsg.cpu_calc_time_microseconds = 0;
    visionMsg.enabled = 0;
    visionMsg.N_obstacles = 1;
    visionMsg.d = zeros(10,1);
    visionMsg.h = zeros(10,1);
    visionMsg.d(1) = 1;
    visionMsg.h(1) = 0.21;
end


lc.publish('CHEETAH_vision_data', visionMsg);