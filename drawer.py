import streamlit as st
import matplotlib.pyplot as plt
from matplotlib.textpath import TextPath
import numpy as np
from math import atan2

# Convert text into coordinates using matplotlib's TextPath
def text_to_coordinates(word, font_size, scale=1.0):
    text_path = TextPath((0, 0), word, size=font_size)
    vertices = text_path.vertices  # Extract x, y points
    codes = text_path.codes  # Extract path commands (moveto, lineto, etc.)

    # Flip Y-axis to align with the robot's coordinate system
    coordinates = np.array(vertices)
    coordinates[:, 1] *= -1

    # Apply scaling
    coordinates *= scale
    return coordinates, codes

# PID Controller class
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, setpoint, actual):
        error = setpoint - actual
        self.integral += error
        derivative = error - self.prev_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

def generate_arduino_code(coordinates, speed, pid_params):
    pid = PIDController(*pid_params)

    arduino_code = f"""
    #include <Servo.h>
    
    #define ENA 5
    #define ENB 6
    #define IN1 7
    #define IN2 8
    #define IN3 9
    #define IN4 11
    
    void move_forward(int speed) {{
        analogWrite(ENA, speed);
        analogWrite(ENB, speed);
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
    }}
    
    void turn_left(int speed) {{
        analogWrite(ENA, speed);
        analogWrite(ENB, speed);
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
    }}
    
    void turn_right(int speed) {{
        analogWrite(ENA, speed);
        analogWrite(ENB, speed);
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
    }}
    
    void stop_motors() {{
        analogWrite(ENA, 0);
        analogWrite(ENB, 0);
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
    }}
    
    void setup() {{
        pinMode(ENA, OUTPUT);
        pinMode(ENB, OUTPUT);
        pinMode(IN1, OUTPUT);
        pinMode(IN2, OUTPUT);
        pinMode(IN3, OUTPUT);
        pinMode(IN4, OUTPUT);
    }}
    
    void loop() {{
    """
    
    prev_x, prev_y = coordinates[0]
    prev_heading = 0  # Initial heading (angle in radians)
    for x, y in coordinates[1:]:
        dx, dy = x - prev_x, y - prev_y
        target_heading = atan2(dy, dx)  # Calculate angle to the next point
        heading_difference = target_heading - prev_heading

        # Calculate distance
        distance = np.sqrt(dx**2 + dy**2)

        # **Increased Delays** (multiply by a factor, e.g., 1.5 for 50% increase)
        move_delay = int((distance / speed) * 5500)  # Forward delay increased by 50%
        turn_delay = int(abs(heading_difference) * 750)  # Turn delay increased by 50%

        if heading_difference > 0:  # Turn right
            arduino_code += f"""
            turn_right({speed});
            delay({turn_delay});  // Increased turn delay
            """
        elif heading_difference < 0:  # Turn left
            arduino_code += f"""
            turn_left({speed});
            delay({turn_delay});  // Increased turn delay
            """

        # Move forward to the next point
        arduino_code += f"""
        move_forward({speed});
        delay({move_delay});  // Increased forward delay
        """

        prev_x, prev_y = x, y
        prev_heading = target_heading

    arduino_code += "    stop_motors();\n    while (true);\n    }\n"
    return arduino_code


# Plot the path in Streamlit
def plot_path(coordinates):
    fig, ax = plt.subplots(figsize=(6, 6))
    x, y = zip(*coordinates)
    ax.plot(x, y, marker="o", color="blue")
    ax.set_title("Robot Drawing Path")
    ax.set_xlabel("X-axis")
    ax.set_ylabel("Y-axis")
    ax.set_aspect("equal", adjustable="box")
    st.pyplot(fig)

# Main Streamlit Application
def main():
    st.title("🤖 Smart Car Drawing Application with Path Controllers")
    st.subheader("🖋️ Generate Arduino Code with Path Turns")

    word = st.text_input("Enter the word to draw:", placeholder="Type a word... 📝")
    font_size = st.slider("Font Size 📏", min_value=10, max_value=50, value=20)
    scale = st.slider("Scaling Factor 🔍", min_value=0.1, max_value=5.0, value=1.0)
    speed = st.slider("Base Speed 🚗", min_value=50, max_value=255, value=100)
    kp = st.number_input("Proportional Gain (Kp) ⚙️", min_value=0.0, value=1.0)
    ki = st.number_input("Integral Gain (Ki) ⚙️", min_value=0.0, value=0.1)
    kd = st.number_input("Derivative Gain (Kd) ⚙️", min_value=0.0, value=0.01)

    if st.button("Generate Path and Code 🚀"):
        if word:
            st.success(f"Generating path and Arduino code for: {word} 🎉")

            # Convert text to coordinates
            coordinates, codes = text_to_coordinates(word, font_size, scale)

            # Plot the path
            plot_path(coordinates)

            # Generate Arduino code
            pid_params = (kp, ki, kd)
            arduino_code = generate_arduino_code(coordinates, speed, pid_params)
            st.code(arduino_code)

            # Allow downloading of the generated code
            st.download_button(
                label="Download Arduino Code 📥",
                data=arduino_code,
                file_name="smart_car_drawing.ino",
                mime="text/plain",
            )
        else:
            st.error("Please enter a word to draw. ❌")

if __name__ == "__main__":
    main()
