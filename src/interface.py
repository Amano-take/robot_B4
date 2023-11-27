import tkinter as tk
import tkinter.ttk as ttk
import rospy
from std_msgs.msg import String

rospy.init_node("interface_target_id")
publisher = rospy.Publisher("/tos_voice_gesture", String, queue_size=1)

root = tk.Tk()
root.geometry("350x1000")
root.title("しゃべる内容")
frame = ttk.Frame(root)
frame.pack(fill = tk.BOTH, padx = 20, pady=20)
txt = tk.Entry(frame, width=20)
txt.pack()
def publish(a=None):
    s = txt.get()
    publisher.publish(s)
    txt.delete(0, tk.END)
txt.bind("<Return>", publish)
bot = tk.Button(frame, text="send", command=publish)
bot.pack()




root.mainloop()