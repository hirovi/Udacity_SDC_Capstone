import threading
from time import sleep


class TLDetector(object):
    def __init__(self):
        print 'tl_detector'
        self.pose = None
        self.num = 1

    def get_light(self):
    	self.num=self.num+1
        print "get_light"+str(self.num)



def runOff():
  threading.Timer(5.0, runOff).start()
  tld.get_light()

if __name__ == '__main__':
    try:
        tld = TLDetector()
        runOff()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')

# def printit():
#   threading.Timer(5.0, printit).start()
#   print "Hello, World!"

# printit()

# while True:
# 	sleep(1)
# 	print "teste"
