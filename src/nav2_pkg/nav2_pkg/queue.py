from collections import deque

# class Queue():
#     def __init__(self):
#         self.queue=[]

#     def enqueue(self,a):
#       self.queue.append(a)
#       return self.queue

#     def is_empty(self):
#        return not self.queue,      #empty lsit is considered false, []=false, not[]=true
       
#     def dequeue(self):
#       if not self.is_empty():
#         front=self.queue[0]
#         self.queue=self.queue[1:]
#         return front
       
    
       
#     def display(self):
#        return self.queue
      
       

    

# a=Queue()
# # print(a.enqueue(14))
# # print(a.enqueue(34))
# # print(a.enqueue(3))
# # print(a.enqueue(4))
# # print(a.enqueue(334))
# # print(a.dequeue())
# # print(a.dequeue())
# # print(a.dequeue())
# # print(a.dequeue())
# # print(a.dequeue())
# # print(a.dequeue())
# # print(a.dequeue())
# # print(a.dequeue())
# # print(a.is_empty())
# # print(a.display())


    
# class Queue2():
#     def __init__(self):
#         self.queue2=deque()
#         self.ss=self.queue2.



# qw=Queue2()
# qw.queue2.append(1)
# qw.queue2.append(2)
# qw.queue2.append(3)
# qw.queue2.append(4)
# qw.queue2.append(5)
# print(qw.queue2)
# qw.queue2.popleft()
# qw.queue2.popleft()
# qw.queue2.popleft()
# print(qw.queue2)
# print(qw.queue2.is)

class bsf():
   def __init__(self):
      
      self.visited={}
      self.queue=deque()
      self.current=None
      self.start=1
  

      self.graph={
                1: [2, 3, 4],
                2: [5, 6],
                3: [7, 8],
                4: [9],
                5: [10],
                6: [11, 12],
                7: [],
                8: [13],
                9: [],
                10: [],
                11: [14],
                12: [],
                13: [15],
                14: [],
                15: []  
           }  
      
   def algo(self):
      self.visited[self.start]=True
      print(f"visited {self.visited}")
      self.queue.append(self.start)
      print(f"queue:{self.queue}")

      while(self.queue):  
        if self.queue: 
          self.current=self.queue[0]
        else:
           break
        print(f"current: {self.current}")
        self.queue.popleft()
        
        print(f"findign neighbour of {self.current}")
        for neighbours in self.graph[self.current]:
          if neighbours not in self.visited:
              self.visited[neighbours]=True
              self.queue.append(neighbours)
              
        print(f"out of loop")
        # self.current=self.queue[0]
        print(f"current {self.current} visited neighbours:{self.visited} queue {self.queue}")   




    




if __name__ == "__main__":
   a = bsf()
   a.algo()
    



    

        
        

    
    



