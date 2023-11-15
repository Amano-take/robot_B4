import tkinter as tk
import tkinter.ttk as ttk
import rospy
from std_msgs.msg import String

rospy.init_node("interface_target_id")
publisher = rospy.Publisher("/target_id", String)

root = tk.Tk()
root.geometry("350x100")
root.title("select target_id")

def publish(i):
    return lambda: publisher.publish(str(i))

frame = ttk.Frame(root)
frame.pack(fill = tk.BOTH, padx = 20, pady=10)

for i in range(1, 11):
    button = tk.Button(frame, text=str(i), command=publish(i))
    button.grid(row=(i-1)//5, column=(i-1)%5)
root.mainloop()