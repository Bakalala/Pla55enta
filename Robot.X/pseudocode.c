/*
 * File:   pseudocode.c
 * Author: aliseifeldin
 *
 * Created on February 6, 2019, 1:35 PM
 */

/*
 * process = false
 * While(process)
 *     Move()
 *     DetectState()
 *     if done
 *          process = false
 * 
 * go to information screen
 
 Move()
 * Turn wheels forward
 * Check Gyro and make adjustments on forward axis if needed
 * Check Both Ultransoic sensors for didstance to canisters at same time and make adjustment if needed
 * Check Encoder distance information and make adjustmenet if needed
 * adjustment --> Make slight changes in wheel speed to make it turn in correct direction  
 * 
 *  If distance == 410
 *      turn(right)
 *      move for 20 cm 
 *      turn(right)  
 * 
 * 
 DetectState()
 * D1 distance of canister at time = t
 * D2 distance of canister at time = t+1
 * 
 *  Diff = D1-D2
 * 
 * If Diff > 20 nothing
 * If Diff = 5 Ball
 *      Stop wheels (after a few seconds to get ball roll right)
 *      Deploy()
 * If Diff = 10 Open
 * If Diff = 0 Closed side
 * 
 * 
 Turn(direction)
 * Stop direction wheel
 * check gyroscope for angle of turn
 * 
 * 
 * 
 Deploy()
 * Turn motor to deploy ball
 * if triggered microswitch
 *  stop motor
 *  
 * 
 * 
 */

        
        
    

