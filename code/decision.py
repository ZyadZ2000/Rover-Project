import numpy as np

def decision_step(Rover):
    offset = 0
    if Rover.total_time > 10:
        offset = 0.8 * np.std(Rover.nav_angles)

    if Rover.nav_angles is not None:
        if Rover.mode[-1] == 'forward':
            if Rover.samples_angles is not None and np.mean(Rover.samples_angles) > -0.2 and np.min(Rover.samples_dists) < 30:
                Rover.rock_time = Rover.total_time
                Rover.mode.append('rock')

            elif len(Rover.nav_angles) >= Rover.step_forward:
                if Rover.vel <= 0.1 and Rover.total_time - Rover.stuck_time > 4:
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode.append('stuck')
                    Rover.stuck_time = Rover.total_time

                elif Rover.vel < Rover.max_vel:
                    Rover.throttle = Rover.throttle_set

                else:
                    Rover.throttle = 0
                Rover.brake = 0

                Rover.steer = np.clip(np.mean((Rover.nav_angles+offset) * 180 / np.pi), -15, 15)

            elif len(Rover.nav_angles) < Rover.stop_forward or Rover.vel <= 0:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode.append('stop')
        
        elif Rover.mode[-1] == 'stuck':

            if Rover.total_time - Rover.stuck_time > 1:
                Rover.throttle = Rover.throttle_set
                Rover.brake = 0
                Rover.steer = np.clip(np.mean((Rover.nav_angles+offset) * 180 / np.pi), -15,15)
                Rover.mode.pop()

                Rover.throttle = 0
                Rover.brake = 0
                Rover.steer = -15

        elif Rover.mode[-1] == 'rock':
            mean = np.mean(Rover.samples_angles * 180 / np.pi)
            if not np.isnan(mean):
                Rover.steer = np.clip(mean, -15, 15)

            else:
                Rover.mode.pop()

            if Rover.total_time - Rover.rock_time > 20:
                Rover.mode.pop()

            if Rover.near_sample:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set

            elif Rover.vel <= 0 and Rover.total_time - Rover.stuck_time > 10:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode.appen('stuck')
                Rover.stuck_time = Rover.total_time

            else:
                slow_speed = Rover.max_vel / 2

                if Rover.vel < slow_speed:
                    Rover.throttle = 0.2
                    Rover.brake = 0
                    
                else:
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set

        elif Rover.mode[-1] == 'stop':
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            
            elif Rover.vel <= 0.2:
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    Rover.brake = 0
                    Rover.steer = -15

                if len(Rover.nav_angles) >= Rover.go_forward:
                    Rover.throttle = Rover.throttle_set
                    Rover.brake = 0
                    offset = 12
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180 / np.pi) + offset, -15, 15)
                    Rover.mode.pop()

        
        else:
            Rover.throttle = Rover.throttle_set
            Rover.steer = 0
            Rover.brake = 0

        if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
            Rover.send_pickup = True

        return Rover
