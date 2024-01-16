import tkinter as tk
from PIL import ImageTk, Image

# Create a window
window = tk.Tk()
# widow name
window.title("注意文言")
# Load the image
image_path = "Paper/img/present.drawio.png"  # Replace with the actual path to your image
image = Image.open(image_path)

# Resize the image if needed
# image = image.resize((width, height))

# Create a Tkinter-compatible photo image
photo = ImageTk.PhotoImage(image)

# Create a label widget to display the image
label = tk.Label(window, image=photo)

# Pack the label widget to the window
label.pack()

# Run the GUI event loop
window.mainloop()
