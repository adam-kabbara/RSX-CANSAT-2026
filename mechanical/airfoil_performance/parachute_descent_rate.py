import matplotlib.pyplot as plt
import numpy as np


# make a function that calculates the descent rate of a parachute given its drag coefficient, area, and massdef calculate_descent_rate(drag_coefficient, area, mass, air_density=1.225):
def calculate_descent_rate(drag_coefficient, area, mass, air_density=1.225):
    """
    Calculate the descent rate of a parachute.

    Parameters:
    drag_coefficient (float): Drag coefficient of the parachute.
    area (float): Area of the parachute in square meters.
    mass (float): Mass of the payload in kilograms.
    air_density (float): Density of air in kg/m^3. Default is 1.225 kg/m^3.

    Returns:
    float: Descent rate in meters per second.
    """
    g = 9.81  # Acceleration due to gravity in m/s^2
    descent_rate = np.sqrt((2 * mass * g) / (air_density * drag_coefficient * area))
    return descent_rate

# make a function that finds the optimal drag coefficient for a parachute given its area and mass
def plot_acceleration_vs_coefficients(drag_coefficients, area, mass, title="Parachute Descent Rate vs Drag Coefficient"):
    """
    Plot descent rate vs drag coefficient for a parachute.

    Parameters:
    drag_coefficients (array-like): Array of drag coefficients to evaluate.
    area (float): Area of the parachute in square meters.
    mass (float): Mass of the payload in kilograms.
    """
    descent_rates = [calculate_descent_rate(cd, area, mass) for cd in drag_coefficients]

    plt.figure(figsize=(10, 6))
    plt.plot(drag_coefficients, descent_rates, marker='o')
    plt.title(title)
    plt.xlabel('Drag Coefficient')
    plt.ylabel('Descent Rate (m/s)')
    plt.grid(True)
    plt.legend(['Mass: {:.2f} kg, Area: {:.2f} m²'.format(mass, area)])
    plt.show()


#make a function to plot velocity over time
def plot_velocity_over_time(drag_coefficients, area, mass, time_duration, time_steps, title="Parachute Velocity Over Time", target_velocity=None):
    """
    Plot velocity over time for a parachute descent.

    Parameters:
    drag_coefficients (float or array-like): Drag coefficient(s) of the parachute. Can be single value or array.
    area (float): Area of the parachute in square meters.
    mass (float): Mass of the payload in kilograms.
    time_duration (float): Total time duration to simulate in seconds.
    time_steps (int): Number of time steps to simulate.
    target_velocity (float, optional): If provided, plots a horizontal dotted line at this velocity.
    """
    g = 9.81  # Acceleration due to gravity in m/s^2
    air_density = 1.225  # Density of air in kg/m^3

    # Handle both single value and array inputs
    if not hasattr(drag_coefficients, '__iter__'):
        drag_coefficients = [drag_coefficients]
    elif hasattr(drag_coefficients, 'shape'):  # numpy array
        drag_coefficients = drag_coefficients.flatten()

    times = np.linspace(0, time_duration, time_steps)

    plt.figure(figsize=(10, 6))
    
    for cd in drag_coefficients:
        velocities = []
        for t in times:
            # Using a simple model where velocity approaches terminal velocity
            terminal_velocity = calculate_descent_rate(cd, area, mass, air_density)
            velocity = terminal_velocity * (1 - np.exp(-g * t / terminal_velocity))
            velocities.append(velocity)
        
        plt.plot(times, velocities, label=f'Cd: {cd:.2f}')

    # Plot target velocity line if provided
    if target_velocity is not None:
        plt.axhline(y=target_velocity, color='red', linestyle='--', linewidth=2, label=f'Target: {target_velocity} m/s')

    plt.title(title)
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.grid(True)
    plt.legend()
    plt.show()


if __name__ == "__main__":
    # x-form parachute
    # min and max estimates of drag coefficient
    # array from 0.6 to 0.8 with 20 values
    drag_coefficients_cansat = np.linspace(0.6, 0.8, 5)
    drag_coefficient_nc = np.linspace(0.75, 0.75, 1)
    s1 = 0.134467**2
    chute_area_cansat = s1+4*s1
    mass_cansat = 1
    mass_nosecone=(67+6.4)*10**-3
    chute_nc_diameter = 12 #inches
    chute_nc_diameter_spill_hole = 40 #mm
    chute_area_nosecone=np.pi*((chute_nc_diameter*0.0254/2)**2-(chute_nc_diameter_spill_hole*0.001/2)**2)

    # for cansat parachute
    print("Cansat Parachute Descent Rates:")
    plot_acceleration_vs_coefficients(drag_coefficients_cansat, chute_area_cansat, mass_cansat, title="Cansat Parachute Descent Rate vs Drag Coefficient")
    plot_velocity_over_time(drag_coefficients_cansat, chute_area_cansat, mass_cansat, time_duration=10, time_steps=100, title="Cansat Parachute Velocity Over Time with Varying Cd", target_velocity=15)
    print("Terminal velocity for Cd=0.78:", calculate_descent_rate(0.78, chute_area_cansat, mass_cansat), "m/s")

    # for nosecone parachute
    print("Nosecone Parachute Descent Rates:")
    plot_acceleration_vs_coefficients(drag_coefficient_nc, chute_area_nosecone, mass_nosecone, title="Nosecone Parachute Descent Rate vs Drag Coefficient")
    plot_velocity_over_time(drag_coefficient_nc, chute_area_nosecone, mass_nosecone, time_duration=10, time_steps=100, title="Nosecone Parachute Velocity Over Time with Varying Cd", target_velocity=5)
    print("Terminal velocity for Cd=0.75:", calculate_descent_rate(0.75, chute_area_nosecone, mass_nosecone), "m/s")