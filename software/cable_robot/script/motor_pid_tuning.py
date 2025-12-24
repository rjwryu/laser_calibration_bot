#!/usr/bin/env python3

import math
import can
import time
from datetime import datetime
from typing import List, Tuple

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from rmd_controller import RMDController

def set_and_collect_feedback(controller: RMDController, control_mode: str, sequence: Tuple[dict]) -> Tuple[List[float], List[float], List[float], List[float]]:
    """
    Execute a sequence of motor commands and collect feedback.

    :param controller: RMDController instance
    :param control_mode: Control mode ('current', 'velocity', 'position')
    :param sequence: Tuple of dicts with 'time' and 'value' keys
    :return: tuple of (timestamps, positions, velocities, currents)
             where each list contains the collected values.
             Timestamps are relative (0 = start of collection).
    """
    timestamps = []
    positions = []
    velocities = []
    currents = []

    start_time = time.time()
    last_feedback_time = start_time
    seq_index = 0
    while seq_index < len(sequence):
        # send move command after specified time in sequence
        elapsed = time.time() - start_time
        if elapsed >= sequence[seq_index]['time']:
            value = sequence[seq_index]['value']
            if value is None:
                print(f"  [{time.time():.6f}] Sequence complete")
                controller.stop_motor()
                break  # end of sequence

            if control_mode == 'position':
                # print(f"[{elapsed:.3f}s] Setting position to {value}°...")
                print(f"  [{time.time():.6f}] Setting absolute position: {value:+.3f}°")
                controller.set_position(value, max_speed=360)
            elif control_mode == 'velocity':
                # print(f"[{elapsed:.3f}s] Setting velocity to {value}dps...")
                print(f"  [{time.time():.6f}] Setting velocity: {value:+.3f} dps")
                controller.set_speed(value)
            elif control_mode == 'current':
                # print(f"[{elapsed:.3f}s] Setting current to {value}A...")
                print(f"  [{time.time():.6f}] Setting current: {value:+.3f} A")
                controller.set_current(value)
            seq_index += 1

        # collect feedback
        now = time.time()
        if now - last_feedback_time >= 0.001:
            feedback = controller.get_motor_feedback()
            if control_mode == 'position':
                hiresPos = controller.get_position()
            
            if feedback:
                pos, vel, cur = feedback
                timestamps.append(elapsed)
                positions.append(hiresPos if control_mode == 'position' else pos)
                velocities.append(vel)
                currents.append(cur)
            last_feedback_time = now

    return timestamps, positions, velocities, currents

def plot_feedback_data(data, seq_for: str, seq: Tuple[dict]) -> None:
    """
    Plot position, velocity, and current time series using matplotlib.
    """
    ts, pos, vel, cur = data

    fig, axes = plt.subplots(3, 1, figsize=(10, 8))

    graph_index = {
        'position': 0,
        'velocity': 1,
        'current':  2,
    }
    cur_graph_index = graph_index[seq_for]

    ref_sig_time = []
    ref_sig_value = []
    for i in range(len(seq)):
        if seq[i]['value'] is None:
            break

        ref_sig_time.append(seq[i]['time'])
        ref_sig_value.append(seq[i]['value'])
        ref_sig_time.append(seq[i+1]['time'] - 1e-9)
        ref_sig_value.append(seq[i]['value'])

    # Reference signal plot
    axes[cur_graph_index].plot(ref_sig_time, ref_sig_value, 'k-', label='Reference Signal')

    # Position plot
    axes[0].plot(ts, pos, 'b-', label='Position')
    axes[0].set_ylabel('Position (degrees)')
    axes[0].set_title('Motor Feedback - Position')
    axes[0].grid(True)
    axes[0].legend()

    # Velocity plot
    axes[1].plot(ts, vel, 'g-', label='Velocity')
    axes[1].set_ylabel('Velocity (dps)')
    axes[1].set_title('Motor Feedback - Velocity')
    axes[1].grid(True)
    axes[1].legend()

    # Current plot
    axes[2].plot(ts, cur, 'r-', label='Current')
    axes[2].set_ylabel('Current (A)')
    axes[2].set_xlabel('Time (seconds)')
    axes[2].set_title('Motor Feedback - Current')
    axes[2].grid(True)
    axes[2].legend()

    plt.tight_layout()
    plt.show()

def step_seq(start_value=0, end_value=1, step_time=1, linger_dur=1):
    return [
        {'time': 0, 'value': start_value},
        {'time': step_time, 'value': end_value},
        {'time': step_time + linger_dur, 'value': None},
    ]

def experiment_R(controller: RMDController, predictors: List[Tuple[float]]) -> dict:
    data = {
        'exp': [],
        'Kp': [],
        'u': [],
        't': [],
        'i': [],
    }

    print("Running R experiments...")
    numExperiments = len(predictors)
    for exp_i in range(numExperiments):
        kp, u = predictors[exp_i]
        print(f"  ({exp_i+1}/{numExperiments}) Kp: {kp}, u: {u}")

        controller.set_pid_params([kp, 0, 0, 0, 0, 0, 0])
        ts, _, _, cur = set_and_collect_feedback(controller, 'current', step_seq(end_value=u))
        controller.stop_motor()

        numSamples = len(ts)
        data['exp'] += [exp_i] * numSamples
        data['Kp'] += [kp] * numSamples
        data['u'] += [u] * numSamples
        data['t'] += ts
        data['i'] += cur
    
    print("Stopping motor...")
    controller.stop_motor()

    return data

def experiment_LR(controller: RMDController, predictors: List[Tuple[float]]) -> dict:
    data = {
        'exp': [],
        'Ki': [],
        'u': [],
        't': [],
        'i': [],
    }

    print("Running LR experiments...")
    numExperiments = len(predictors)
    for exp_i in range(numExperiments):
        ki, u = predictors[exp_i]
        print(f" ({exp_i+1}/{numExperiments}) Ki: {ki}, u: {u}")

        controller.set_pid_params([0.5,0.1,0,0,0,0,0])
        time.sleep(0.1)
        controller.set_current(u/2)
        time.sleep(0.1)

        controller.set_pid_params([0, ki, 0, 0, 0, 0, 0])
        ts, _, _, cur = set_and_collect_feedback(controller, 'current', step_seq(u/2, u, 0.5, 1))
        controller.stop_motor()

        numSamples = len(ts)
        data['exp'] += [exp_i] * numSamples
        data['Ki'] += [ki] * numSamples
        data['u'] += [u] * numSamples
        data['t'] += ts
        data['i'] += cur
    
    print("Stopping motor...")
    controller.stop_motor()

    return data

def save_csv(data: dict, name: str) -> None:
    dt = datetime.now().strftime("%Y-%m-%dT%H%M")
    name = f"data/{name}_{dt}.csv"
    print(f"Saving to file '{name}'...")
    pd.DataFrame(data).to_csv(name, index=False)

def find_custom_response(controller: RMDController, pid_params: List[float], control_mode: str, sequence):
    print("Setting PID parameters:", pid_params)
    time.sleep(0.5)   # don't set pid params too quickly after bus open
    controller.set_pid_params(pid_params)
    
    print(f"Control mode: {control_mode}")

    print(f"Collecting motor feedback...")
    timestamps, positions, velocities, currents = set_and_collect_feedback(controller, control_mode, sequence)

    print(f"Collected {len(timestamps)} samples")
    if timestamps:
        print(f"  Position range: {min(positions):.2f}° to {max(positions):.2f}°")
        print(f"  Velocity range: {min(velocities):.1f} to {max(velocities):.1f} dps")
        print(f"  Current range: {min(currents):.3f} to {max(currents):.3f} A")

    print("Stopping motor...")
    controller.stop_motor()
    print("Shutting down CAN bus...")
    controller.bus.shutdown()

    if timestamps:
        print("Plotting data...")
        plot_feedback_data(timestamps, positions, velocities, currents)
    else:
        print("No feedback data collected; skipping plot.")

def main():
    # Configuration
    channel = 'can0'
    motor_id = 4
    control_mode = 'position'  # options: 'current', 'velocity', 'position'
    # pid_params = [0.5, 0.1, 0.03, 0.001, 1.0, 0.1, 0.5]   # OG params
    # pid_params = [0.5, 0.1, 0.03, 0.001, 0.05, 0.00001, 0.01]          # best vibration
    # pid_params = [2.4, 0.2, 0.05, 0.001, 0.1, 0.001, 0.1]            # best disturbance rejection
    pid_params = [0.3, 0.03, 0.08, 0.001, 0.05, 0.00001, 0.01]          # the one
    max_acc = [10000]*4        # OG: [10000]*4
    start_val = 0
    step_val = 180
    time_start = 0.5
    time_step = 2
    seq = [
        # {'time': 0, 'value': 50},
        # {'time': 1, 'value': 100},
        # {'time': 2, 'value': 50},
        # {'time': 3, 'value': None}
        {'time': time_start+time_step*0, 'value': start_val},
        {'time': time_start+time_step*1, 'value': -step_val},
        {'time': time_start+time_step*2, 'value': step_val},
        {'time': time_start+time_step*3, 'value': start_val},
        {'time': time_start+time_step*4, 'value': None}
    ]
    # amp = 30
    # period = 1
    # signal_dur = 2
    # signal_freq = 50
    # seq = [{'time': t, 'value': amp*math.sin(math.tau * t/period)} for t in np.linspace(0, signal_dur, num=signal_freq * signal_dur)]
    # seq[-1]['value'] = None

    # range_Kp = 1 / np.linspace(1/1, 1/5, num=10)
    # range_Ki = np.linspace(0.0002, 0.0004, num=10)
    # range_u = np.linspace(5, 10, num=3)

    print("=== Motor PID Tuning ===")
    print(f"Opening CAN bus on channel {channel}...")
    with can.Bus(channel=channel, interface='socketcan') as bus:
        print(f"Creating RMDController for motor 0x{motor_id:x}...")
        controller = RMDController(motor_id, bus)
        print()

        print(f"Absolute position (before zero): {controller.get_position():+.3f}°")
        print(f"Original max accels: {controller.get_max_acc()}")
        print(f"Original PID parameters: {controller.get_pid_params()}")
        print()

        print("Saving PID parameters... ", end='')
        controller.save_pid_params(pid_params)
        time.sleep(0.2)
        print('Done')
        print(f"New PID parameters: {controller.get_pid_params()}")
        print()

        

        # print("Resetting zero position... ", end='')
        # controller.reset_zero_pos()
        # time.sleep(0.2)
        # print('Done')
        # print(f"Absolute position (after zero): {controller.get_position():+.3f}°")
        # print()

        # print('Setting new max accels... ', end='')
        # controller.set_max_acc(max_acc)
        # time.sleep(0.2)
        # print('Done')
        # print(f"New max accels: {controller.get_max_acc()}")
        # print()

        # print("Setting new PID parameters... ", end='')
        # controller.set_pid_params(pid_params)
        # time.sleep(0.2)
        # print('Done')
        # print(f"New PID parameters: {controller.get_pid_params()}")
        # print()

        # while True:
        #     try:
        #         num = float(input('Enter new absolute position (deg): '))
        #         controller.set_position(num, max_speed=360)
        #     except ValueError:
        #         controller.stop_motor()
        #         break

        # print("Running sequence... ")
        # data = set_and_collect_feedback(controller, control_mode, seq)
        # print("Sequence complete.")
        # print()

        # print('Plotting data... ')
        # plot_feedback_data(data, control_mode, seq)

        # exp_LR_predictors = [(ki, u) for ki in range_Ki for u in range_u]
        # exp_LR_data = experiment_LR(controller, exp_LR_predictors)
        # save_csv(exp_LR_data, 'exp_LR')

        # controller.set_pid_params([1, 0, 0, 0, 0, 0, 0])
        # ts, _, _, cur = set_and_collect_feedback(controller, 'current', step_seq(end_value=5))
        # controller.stop_motor()

        # for i in range(len(ts)):
        #     print(f"{ts[i]:.6f}, {cur[i]:.6f}")
        
        # exp_R_predictors = [(kp, u) for kp in range_Kp for u in range_u]
        # exp_R_data = experiment_R(controller, exp_R_predictors)
        # save_csv(exp_R_data, 'exp_R')

        # find_custom_response(controller, pid_params, "current", sequence)

    print("CAN bus shut down successfully")
    print("=== PID Tuning Complete ===")
if __name__ == '__main__':
    main()