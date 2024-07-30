import tkinter as tk


class App(tk.Tk):
    # 呪文
    def __init__(self, *args, **kwargs):
        # 呪文
        tk.Tk.__init__(self, *args, **kwargs)

        # ウィンドウタイトルを決定
        self.title("注意文言")

        # ウィンドウの大きさを決定
        self.geometry("300x150")

        # ウィンドウのグリッドを 1x1 にする
        # この処理をコメントアウトすると配置がズレる
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=1)
#-----------------------------------main_frame-----------------------------
        # メインページフレーム作成
        self.main_frame = tk.Frame()
        self.main_frame.grid(row=0, column=0, sticky="nsew")
        self.main_frame.grid_rowconfigure(0, weight=2)
        self.main_frame.grid_columnconfigure(0, weight=1)
        self.main_frame.grid_rowconfigure(1, weight=1)
        self.main_frame.grid_columnconfigure(1, weight=1)
        self.main_frame.grid_rowconfigure(2, weight=1)
        self.main_frame.grid_columnconfigure(2, weight=1)


        # タイトルラベル作成
        self.Label1 = tk.Label(self.main_frame, text="「道案内ならわたしがしますよ。」")
        self.Label1.grid(row=0, column=0, columnspan=2)
        self.respondLabel = tk.Label(self.main_frame, text="歩きスマホをやめた場合")
        self.respondLabel.grid(row=1, column=0)
        self.ansRespondButton = tk.Button(self.main_frame, text="Go", command=lambda : self.changePage(self.frame1))
        self.ansRespondButton.grid(row=2, column=0)
        self.respondLabel = tk.Label(self.main_frame, text="歩きスマホをやめなかった場合")
        self.respondLabel.grid(row=1, column=1)
        self.changePageButton = tk.Button(self.main_frame, text="Go", command=lambda : self.changePage(self.frame1))
        self.changePageButton.grid(row=2, column=1)
#--------------------------------------------------------------------------
#-----------------------------------frame1---------------------------------
        # 移動先フレーム作成
        self.frame1 = tk.Frame()
        self.frame1.grid(row=0, column=0, sticky="nsew")
        # タイトルラベル作成
        self.titleLabel = tk.Label(self.frame1, text="Frame 1", font=('Helvetica', '35'))
        self.titleLabel.pack(anchor='center', expand=True)
        # フレーム1からmainフレームに戻るボタン
        self.back_button = tk.Button(self.frame1, text="Back", command=lambda : self.changePage(self.main_frame))
        self.back_button.pack()
#--------------------------------------------------------------------------
#-----------------------------------frame2---------------------------------


        #main_frameを一番上に表示
        self.main_frame.tkraise()

    def changePage(self, page):
        '''
        画面遷移用の関数
        '''
        page.tkraise()

if __name__ == "__main__":
    app = App()
    app.mainloop()
