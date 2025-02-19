from mirs_system.ai.vision.calibration.raspberrypi import Raspberrypi


rasp = Raspberrypi()

out,err = rasp.execute("ls")

print("Exit status : ",out.channel.recv_exit_status())
print("\n\nTest output :")

for line in out:
    print(line)

print("\n\n")
print("Test error :")

for line in err.readlines():
    print(line)