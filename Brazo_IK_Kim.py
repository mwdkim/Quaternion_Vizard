#Asssignment sample script
# written by Almudena Blasco Delgado, September 23, 2023
#modified by Wangdo Kim
import viz
import vizcam
import csv
import viztask
import math
import vizconnect
#izconnect.go("vizconnect_config_oculus_1.py")
viz.go()
# Load your environment and character model in .osgb format
# env = viz.addChild('Bedroom_1/Bedroom_1.osgb')
env = viz.addChild('Environments/serving_room.osgb')
character = viz.addAvatar('Avatar/vcc_male.cfg')
vizcam.PivotNavigate(center=[0, 1.8, 0], distance=3)

# Define a dictionary to map bone names to their corresponding CSV files
bone_files = {
    'Bip01 R UpperArm': 'hombro.csv',
    'Bip01 R Forearm': 'antebrazo.csv',
    'Bip01 R Hand': 'muneca.csv',
}

# Function to apply quaternion and acceleration data to a specified bone
def apply_bone_data(bone_name, row):
    try:
        w, x, y, z = map(float, [row['Quat_W'], row['Quat_X'], row['Quat_Y'], row['Quat_Z']])
        quat = viz.Quat(w, x, y, z)
        correction = viz.Quat(1.0, 0.0, 0.0, math.radians(90.0))
        quaternion = correction * quat

        acc_x, acc_y, acc_z = map(float, [row['FreeAcc_X'], row['FreeAcc_Y'], row['FreeAcc_Z']])
        scale_factor = 0.01 

        # Apply a 90-degree transformation to the acceleration data
        acc_x, acc_y, acc_z = math.radians(90.0), math.radians(90.0), math.radians(90.0)

        # Get the bone
        bone = character.getBone(bone_name)

        # Update the bone's position based on acceleration data
#        new_position = bone.getPosition() + viz.Vector(acc_x, acc_y, acc_z) * scale_factor
#        bone.setPosition(new_position)

        # Update the bone's orientation based on quaternion data
        bone.setQuat(quaternion)

    except ValueError:
        # Skip rows that cannot be converted to float (e.g., empty rows)
        return

# Function to read CSV data
def read_csv(csv_file):
    data = []
    with open(csv_file, 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            data.append(row)
    return data

# Create a task to apply quaternion and acceleration data to the entire arm
def apply_arm_data(upper_arm_file, forearm_file, hand_file):
    upper_arm_data = read_csv(upper_arm_file)
    forearm_data = read_csv(forearm_file)
    hand_data = read_csv(hand_file)

    for upper_arm_row, forearm_row, hand_row in zip(upper_arm_data, forearm_data, hand_data):
        apply_bone_data('Bip01 R UpperArm', upper_arm_row)
        apply_bone_data('Bip01 R Forearm', forearm_row)
        apply_bone_data('Bip01 R Hand', hand_row)
        yield viztask.waitTime(0.1)

# Create a task for applying arm data
arm_task = apply_arm_data(bone_files['Bip01 R UpperArm'], bone_files['Bip01 R Forearm'], bone_files['Bip01 R Hand'])

# Schedule the task
viztask.schedule(arm_task)

# Start the Vizard scene
viz.go()