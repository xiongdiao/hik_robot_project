#!/usr/bin/env python

from pi_trees_lib.pi_trees_lib import *
import time

class RobotFuncTask():
    def __init__(self):
        # The root node
        XdBtree = Sequence("xd_btree")
        
        # Create a Selector composite task (returns SUCCESS as soon as any subtask returns SUCCESS)
        SelectorNode = Selector("SelectorNode")

        # Create a Sequence composite task (returns FAULURE as soon as any subtask returns FAILURE)
        SequenceNode = Sequence("SequenceNode")
        
        # Create three counting tasks
        Count2 = Count("Count+2", 1, 2, 1)
        Count5 = Count("Count-5", 5, 1, -1)
        Count6 = Count("Count-6", 6, 1, -1)
        Count16 = Count("Count+16", 1, 16, 1)

        # Add the tasks to the sequence composite task
        SequenceNode.add_child(Count5)
        SequenceNode.add_child(Count2)
        
        # Add the tasks to the selector composite task
        SelectorNode.add_child(Count16)
        SelectorNode.add_child(Count6)
        
        # Add the composite task to the root task
        XdBtree.add_child(SequenceNode)
        XdBtree.add_child(SelectorNode)

        # Print a simple representation of the tree
        print "Behavior Tree Structure"
        print_tree(XdBtree, use_symbols=True)

        status = TaskStatus.RUNNING    

        # Run the tree
        while status != TaskStatus.FAILURE:
            status = XdBtree.run()

            if status == TaskStatus.SUCCESS:
                print "Finished running tree."
                break
            elif status == TaskStatus.RUNNING:
                print "Running status"

            time.sleep(0.02)

# A counting task that extends the base Task task
class Count(Task):
    def __init__(self, name, start, stop, step, *args, **kwargs):
        super(Count, self).__init__(name, *args, **kwargs)
        
        self.name = name
        self.start = start
        self.stop = stop
        self.step = step
        self.count = self.start
        print "Creating task Count", self.start, self.stop, self.step
 
    def run(self):
        if abs(self.count - self.stop - self.step) <= 0:
            print self.name, "returen sucess"
            return TaskStatus.SUCCESS
        else:
            print self.name, self.count
            #time.sleep(0.1)

            if self.count >= 10:
                #print self.name, "return Failure"
                return TaskStatus.FAILURE

            self.count += self.step

            if abs(self.count - self.stop - self.step) <= 0:
                print self.name, "returen sucess"
                return TaskStatus.SUCCESS
            else:
                return TaskStatus.RUNNING

    
    def reset(self):
        self.count = self.start

if __name__ == '__main__':
    tree = RobotFuncTask()





