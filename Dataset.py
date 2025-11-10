f = open("data.txt", "w+")
# Write sensor data to text file
f.write(str(col_left.value()) + "," + str(col_mid.value()) + "," + str(col_right.value()) + "," + str(left_motor.speed) + "," + str(right_motor.speed) + "\n")
