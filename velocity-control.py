import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

class VelocityControlLoop:
    def __init__(self, simulation_time=10.0, dt=0.001):
        # Simulation parameters
        self.dt = dt
        self.t = np.arange(0, simulation_time, dt)
        
        # Initialize arrays for storing results
        self.w_desired = np.zeros_like(self.t)
        self.w_actual = np.zeros_like(self.t)
        self.w_error = np.zeros_like(self.t)
        self.torque = np.zeros_like(self.t)
        self.integral_term = np.zeros_like(self.t)
        
        # Controller parameters
        self.Kp = 10.0  # Proportional gain
        self.Ki = 5.0   # Integral gain
        self.max_torque = 20.0  # Torque saturation limit
        
        # Generate desired speed trajectory (sinusoidal in this case)
        self.w_desired = 2.0 * np.sin(2.0 * np.pi * 0.5 * self.t)
        
        # Disturbance torque (constant in this case)
        self.tau_d = np.zeros_like(self.t)
        self.tau_d[int(len(self.t)/3):int(2*len(self.t)/3)] = 2.0  # Apply disturbance in middle third
        
        # System parameters
        self.J = 0.1  # Moment of inertia
        
    def saturate(self, value):
        """Implement torque saturation"""
        return np.clip(value, -self.max_torque, self.max_torque)
    
    def system_dynamics(self, state, t, torque, tau_d):
        """Define system dynamics: dw/dt = (torque + tau_d) / J"""
        return (torque + tau_d) / self.J
    
    def simulate(self):
        integral = 0.0  # Initialize integral term
        w = 0.0        # Initialize angular velocity
        
        for i in range(1, len(self.t)):
            # Calculate error
            error = self.w_desired[i-1] - w
            self.w_error[i-1] = error
            
            # Update integral term
            integral += error * self.dt
            self.integral_term[i-1] = integral
            
            # Calculate control input (PI controller)
            tau_ff = 0.0  # Feedforward term (zero in this case)
            tau = self.Kp * error + self.Ki * integral + tau_ff
            
            # Apply saturation
            tau_saturated = self.saturate(tau)
            self.torque[i-1] = tau_saturated
            
            # Simulate system dynamics
            w_dot = self.system_dynamics(w, self.t[i-1], tau_saturated, self.tau_d[i-1])
            w += w_dot * self.dt  # Euler integration
            
            # Store results
            self.w_actual[i-1] = w
            
        # Store final values
        self.w_error[-1] = self.w_desired[-1] - w
        self.integral_term[-1] = integral
        self.torque[-1] = tau_saturated
        self.w_actual[-1] = w
    
    def plot_results(self):
        """Plot all relevant signals"""
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
        
        # Plot desired and actual joint angle
        ax1.plot(self.t, self.w_desired, 'b-', label='Desired')
        ax1.plot(self.t, self.w_actual, 'r--', label='Actual')
        ax1.set_title('Joint Angular Velocity')
        ax1.set_xlabel('Time [s]')
        ax1.set_ylabel('Angular Velocity [rad/s]')
        ax1.grid(True)
        ax1.legend()
        
        # Plot error
        ax2.plot(self.t, self.w_error)
        ax2.set_title('Velocity Error')
        ax2.set_xlabel('Time [s]')
        ax2.set_ylabel('Error [rad/s]')
        ax2.grid(True)
        
        # Plot torque
        ax3.plot(self.t, self.torque)
        ax3.set_title('Motor Torque')
        ax3.set_xlabel('Time [s]')
        ax3.set_ylabel('Torque [Nm]')
        ax3.grid(True)
        
        # Plot integral term
        ax4.plot(self.t, self.integral_term)
        ax4.set_title('Integral Term')
        ax4.set_xlabel('Time [s]')
        ax4.set_ylabel('Integral')
        ax4.grid(True)
        
        plt.tight_layout()
        plt.show()

# Create and run simulation
sim = VelocityControlLoop()
sim.simulate()
sim.plot_results()