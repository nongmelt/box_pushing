import serial
import csv

# csv file name
filename = "result1.csv"


def main():
    count = 0
    with open(filename, "w") as csvfile:
        with serial.Serial("/dev/cu.usbmodem1301", 9600) as ser:
            while True:
                line = ser.readline().decode("utf-8")
                if line.startswith(":"):
                    count += 1
                    line = line[1:]
                if count > 1:
                    break
                if count == 1:
                    csvfile.write(line)


if __name__ == "__main__":
    main()
