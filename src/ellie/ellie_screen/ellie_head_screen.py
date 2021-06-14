from tkinter import * 
from PIL import Image
import PIL.ImageTk as ImageTk

class EllieHeadScreenAni(Label):

    def __init__(self, master, filename):
        self.im = Image.open(filename)
        seq =  []
        try:
            while 1:
                seq.append(self.im.resize((800,480)).copy())
                self.im.seek(len(seq)) # skip to next frame
        except EOFError:
            pass # we're done

        try:
            self.delay = self.im.info['duration']
        except KeyError:
            self.delay = 100

        first = seq[0].convert('RGBA')
        self.frames = [ImageTk.PhotoImage(first)]

        Label.__init__(self, master, image=self.frames[0])

        temp = seq[0]
        for image in seq[1:]:
            temp.paste(image)
            frame = temp.convert('RGBA')
            self.frames.append(ImageTk.PhotoImage(frame))

        self.idx = 0

        self.cancel = self.after(self.delay, self.play)

    def set_image(self,file_name):
        self.im = Image.open(file_name)
        seq =  []
        try:
            while 1:
                seq.append(self.im.resize((800,480)).copy())
                self.im.seek(len(seq)) # skip to next frame
        except EOFError:
            pass # we're done

        try:
            self.delay = self.im.info['duration']
        except KeyError:
            self.delay = 100

        first = seq[0].convert('RGBA')
        self.frames = [ImageTk.PhotoImage(first)]

    def play(self):
        self.config(image=self.frames[self.idx],justify="left")
        self.idx += 1
        if self.idx == len(self.frames):
            self.idx = 0
        self.cancel = self.after(self.delay, self.play)        

class EllieHeadScreen:
    def __init__(self) :
        self.root = Tk()
        self.anim = EllieHeadScreenAni(self.root, 'eyes.gif')
        self.anim.pack()
        self.root.attributes('-fullscreen', True)
        self.root.overrideredirect(True)
        l =["eyes.gif","GAwl.gif"]
        #Button(root, text='stop', command=stop_it).pack()
        self.change_image(self.anim, l, 1) 
        self.root.mainloop()

    def stop_it(self):
        self.anim.after_cancel(self.anim.cancel)
    def change_image(self,label, imagelist, nextindex):
        label.set_image(imagelist[nextindex])
        self.root.after(2000, lambda: self.change_image(label, imagelist, (nextindex+1) % len(imagelist)))

if __name__=="__main__":
    hs = EllieHeadScreen()
