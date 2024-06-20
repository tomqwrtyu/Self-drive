C++  JLL  2021.4.6 -
Copy and Paste Code Snippets to z.cpp
For reading, checking, learning, and running OP code:
--------------------------------------------------

CCCCCCCCCCCCCCCCCCCCCCCC

CCCCCCCCCCCCCCCCCCCCCCCC
/*
C++ 22  2023.5.3
for 230505 Q1

from 4.9.1 Another example involving painting on PyQ5 widgets
--- do not run this only for reading
*/
import sys
from PyQt5 import QtGui, QtWidgets
from PyQt5.QtCore import Qt, QPoint
class MyWidget(QtWidgets.QWidget):
    def paintEvent(self, event):
        qp = QtGui.QPainter()
        qp.begin(self)
        qp.setPen(QtGui.QColor(200,0,0))
        qp.drawText(20,20, "Text at fixed coordinates")
        qp.drawText(event.rect(), Qt.AlignCenter, "Text centered in the drawing area")
        qp.setPen(QtGui.QPen(Qt.darkGreen, 4))
        qp.drawEllipse(QPoint(50,60),30,30)
        qp.setPen(QtGui.QPen(Qt.blue, 2, join = Qt.MiterJoin))
        qp.drawRect(20,60,50,80)
        qp.end()
app = QtWidgets.QApplication(sys.argv)
window = MyWidget()
window.show()
sys.exit(app.exec_())

CCCCCCCCCCCCCCCCCCCCCCCC
/*
C++ 21  2023.3.16
for avcodec_send_packet(decoder_ctx, pkt), CB2.py 230311

from
 * Copyright (c) 2001 Fabrice Bellard
   https://ffmpeg.org/doxygen/3.4/decode_video_8c-example.html#a10
 * @file
 * video decoding with libavcodec API example
 *
 * @example decode_video.c

--- do not run this only for reading
 jinn@Liu:~/openpilot/acpp$ g++ z.cpp -std=c++14 -o z
 collect2: error: ld returned 1 exit status
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libavcodec/avcodec.h>
#define INBUF_SIZE 4096
static void pgm_save(unsigned char *buf, int wrap, int xsize, int ysize,
                     char *filename)
{
    FILE *f;
    int i;
    f = fopen(filename,"w");
    fprintf(f, "P5\n%d %d\n%d\n", xsize, ysize, 255);
    for (i = 0; i < ysize; i++)
        fwrite(buf + i * wrap, 1, xsize, f);
    fclose(f);
}
static void decode(AVCodecContext *dec_ctx, AVFrame *frame, AVPacket *pkt,
                   const char *filename)
{
    char buf[1024];
    int ret;
    ret = avcodec_send_packet(dec_ctx, pkt);
    if (ret < 0) {
        fprintf(stderr, "Error sending a packet for decoding\n");
        exit(1);
    }
    while (ret >= 0) {
        ret = avcodec_receive_frame(dec_ctx, frame);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
            return;
        else if (ret < 0) {
            fprintf(stderr, "Error during decoding\n");
            exit(1);
        }
        printf("saving frame %3d\n", dec_ctx->frame_number);
        fflush(stdout);
        /* the picture is allocated by the decoder. no need to
           free it */
        snprintf(buf, sizeof(buf), "%s-%d", filename, dec_ctx->frame_number);
        pgm_save(frame->data[0], frame->linesize[0],
                 frame->width, frame->height, buf);
    }
}
int main(int argc, char **argv)
{
    const char *filename, *outfilename;
    const AVCodec *codec;
    AVCodecParserContext *parser;
    AVCodecContext *c= NULL;
    FILE *f;
    AVFrame *frame;
    uint8_t inbuf[INBUF_SIZE + AV_INPUT_BUFFER_PADDING_SIZE];
    uint8_t *data;
    size_t   data_size;
    int ret;
    AVPacket *pkt;
    if (argc <= 2) {
        fprintf(stderr, "Usage: %s <input file> <output file>\n", argv[0]);
        exit(0);
    }
    filename    = argv[1];
    outfilename = argv[2];
    avcodec_register_all();
    pkt = av_packet_alloc();
    if (!pkt)
        exit(1);
    /* set end of buffer to 0 (this ensures that no overreading happens for damaged MPEG streams) */
    memset(inbuf + INBUF_SIZE, 0, AV_INPUT_BUFFER_PADDING_SIZE);
    /* find the MPEG-1 video decoder */
    codec = avcodec_find_decoder(AV_CODEC_ID_MPEG1VIDEO);
      // Find a registered decoder with a matching codec ID
      // enum AVCodecID { ...  AV_CODEC_ID_MPEG1VIDEO
    if (!codec) {
        fprintf(stderr, "Codec not found\n");
        exit(1);
    }
    parser = av_parser_init(codec->id);
    if (!parser) {
        fprintf(stderr, "parser not found\n");
        exit(1);
    }
    c = avcodec_alloc_context3(codec);
      // Allocate an AVCodecContext and set its fields to default values.
    if (!c) {
        fprintf(stderr, "Could not allocate video codec context\n");
        exit(1);
    }
    /* For some codecs, such as msmpeg4 and mpeg4, width and height
       MUST be initialized there because this information is not
       available in the bitstream. */
    /* open it */
    if (avcodec_open2(c, codec, NULL) < 0) {
        fprintf(stderr, "Could not open codec\n");
        exit(1);
    }
    f = fopen(filename, "rb");
    if (!f) {
        fprintf(stderr, "Could not open %s\n", filename);
        exit(1);
    }
    frame = av_frame_alloc();
    if (!frame) {
        fprintf(stderr, "Could not allocate video frame\n");
        exit(1);
    }
    while (!feof(f)) {
        /* read raw data from the input file */
        data_size = fread(inbuf, 1, INBUF_SIZE, f);
        if (!data_size)
            break;
        /* use the parser to split the data into frames */
        data = inbuf;
        while (data_size > 0) {
            ret = av_parser_parse2(parser, c, &pkt->data, &pkt->size,
                                   data, data_size, AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
            if (ret < 0) {
                fprintf(stderr, "Error while parsing\n");
                exit(1);
            }
            data      += ret;
            data_size -= ret;
            if (pkt->size)
                decode(c, frame, pkt, outfilename);
        }
    }
    /* flush the decoder */
    decode(c, frame, NULL, outfilename);
    fclose(f);
    av_parser_close(parser);
    avcodec_free_context(&c);
    av_frame_free(&frame);
    av_packet_free(&pkt);
    return 0;
}
https://ffmpeg.org/doxygen/3.4/decode_8c_source.html
646 int attribute_align_arg avcodec_send_packet(AVCodecContext *avctx, const AVPacket *avpkt)
647 {
648     AVCodecInternal *avci = avctx->internal;
649     int ret;
651     if (!avcodec_is_open(avctx) || !av_codec_is_decoder(avctx->codec))
652         return AVERROR(EINVAL);
654     if (avctx->internal->draining)
655         return AVERROR_EOF;
657     if (avpkt && !avpkt->size && avpkt->data)
658         return AVERROR(EINVAL);
660     ret = bsfs_init(avctx);
661     if (ret < 0)
662         return ret;
664     av_packet_unref(avci->buffer_pkt);
665     if (avpkt && (avpkt->data || avpkt->side_data_elems)) {
666         ret = av_packet_ref(avci->buffer_pkt, avpkt);
667         if (ret < 0)
668             return ret;
669     }
671     ret = av_bsf_send_packet(avci->filter.bsfs[0], avci->buffer_pkt);
672     if (ret < 0) {
673         av_packet_unref(avci->buffer_pkt);
674         return ret;
675     }
677     if (!avci->buffer_frame->buf[0]) {
678         ret = decode_receive_frame_internal(avctx, avci->buffer_frame);
679         if (ret < 0 && ret != AVERROR(EAGAIN) && ret != AVERROR_EOF)
680             return ret;
681     }
683     return 0;
684 }

CCCCCCCCCCCCCCCCCCCCCCCC
/*
C++ 20  2023.3.15
for 230311: read_frame = [&](FrameReader *fr, int frame_id)
*/
#include <iostream>
#include <string>
using namespace std;
int main() {
  auto read_frame = [&](bool c) {
    string x = "X";  // OK.
    string *y = &x;
  	cout << "//--- y = " << y << ", &y = " << &y << ", *y = " << *y << endl;
      //--- y = 0x7ffe838b6e30, &y = 0x7ffe838b6e28, *y = X
    return c ? y : nullptr;
  };
  //auto yuv = read_frame(true);
  auto yuv = read_frame(false);
  if (yuv) {
  	string z = *yuv;
  	cout << "//--- true yuv = " << yuv << ", true z = " << z << endl;
      //--- true yuv = 0x7ffe838b6e30, true z = X
  } else {
  	string *z = yuv;
    cout << "//--- false yuv = " << yuv << ", false z = " << z << endl;
      //--- false yuv = 0, false z = 0
  }
  return 0;
}
/*
W3 output:
  y = 0x7ffff6f73ce0, &y = 0x7ffff6f73cd8, *y = X
  true yuv = 0x7ffff6f73ce0, true z = X
PC output: OK.
  y = 0x7ffc823035e0, &y = 0x7ffc823035d8, *y = X
  true yuv = 0x7ffc823035e0, true z = X
*/
CCCCCCCCCCCCCCCCCCCCCCCC
/*
C++ 19  2023.3.15
for 230311: read_frame = [&](FrameReader *fr, int frame_id)
*/
#include <iostream>
using namespace std;
int main() {
  auto read_frame = [&](bool c) {
    char x = 'X';  // NG: char x = "X"
    char *y = &x;
  	cout << "y = " << y << ", &y = " << &y << ", *y = " << *y << endl;
    return c ? y : nullptr;
  };
  auto yuv = read_frame(true);
  //auto yuv = read_frame(false);
  if (yuv) {
  	char z = *yuv;
  	cout << "true &yuv = " << &yuv << ", true z = " << z << endl;
  } else {
  	char *z = yuv;
  	cout << "true &yuv = " << &yuv << ", true z = " << z << endl;
  }
  return 0;
}
/*
W3 output:
  y = X, &y = 0x7ffe654b6500, *y = X
  true &yuv = 0x7ffe654b6500, true z = X
PC output: ???
  y = X�����, &y = 0x7ffde1b69ac0, *y = X
  true &yuv = 0x7ffde1b69ae8, true z = X
*/
CCCCCCCCCCCCCCCCCCCCCCCC
/*
C++ 18  2023.3.12
for ConsoleUI::initWindows(), CB2.py 230311
from https://stackoverflow.com/questions/63026448/ncurses-mvwprintw-doesnt-print

jinn@Liu:~/openpilot/acpp$ gcc z.cpp -lncurses
jinn@Liu:~/openpilot/acpp$ ./a.out
  NG. mvwprintw does not print.
*/
#include <ncurses.h>  /* ncurses.h includes stdio.h */
int main() {
  initscr();
  noecho();
  int scrx, scry;
  getmaxyx(stdscr, scry, scrx);
  WINDOW *w = newwin(1, scrx, scry - 1, 0);
   // WINDOW *newwin(int nlines, int ncols, int begin_y, int begin_x);
  mvwprintw(w, 2, 3, "openpilot replay %s", "0.9.1");
    // mvwprintw(Window, Line, Column, Format, [Argument ...])
  getch();  // shows the window
  delwin(w);
  endwin();
  return 0;
}
CCCCCCCCCCCCCCCCCCCCCCCC
/*
C++ 17  2023.3.12
for ConsoleUI::initWindows(), CB2.py 230311
form NCURSES Programming HOWTO

jinn@Liu:~/openpilot/acpp$ gcc z.cpp -lncurses
jinn@Liu:~/openpilot/acpp$ ./a.out
  OK. mvprintw prints.
*/
#include <ncurses.h>  /* ncurses.h includes stdio.h */
#include <string.h>
int main()
{
  char mesg[]="Just a string";  /* message to be appeared on the screen */
  int row,col;  /* to store the numbers of rows and colums of the screen */
  initscr();	 /* start the curses mode */
  getmaxyx(stdscr,row,col);	/* get the number of rows and columns */
  mvprintw(row/2,(col-strlen(mesg))/2,"%s",mesg);
    /* print the message at the center of the screen */
  mvprintw(row-2,0,"This screen has %d rows and %d columns\n",row,col);
  printw("Try resizing your window(if possible) and then run this program again");
  refresh();
  getch();
  endwin();
  return 0;
}
CCCCCCCCCCCCCCCCCCCCCCCC
/*
C++ 16  2023.3.11
for ConsoleUI::initWindows(), CB2.py 230311
from NCURSES Programming HOWTO

jinn@Liu:~/openpilot/acpp$ gcc z.cpp -lncurses
jinn@Liu:~/openpilot/acpp$ ./a.out
  OK. printw prints. NG. mvwprintw does not print.
*/
#include <stdio.h>  // for printf()
#include <array>  // for std::array
#include <tuple>  // for std::tuple
#include <ncurses.h>
enum Win { Title, Stats, Log, LogBorder, DownloadBar, Timeline, TimelineDesc, Help, CarState, Max};
enum Color { Default, Debug, Yellow, Green, Red, Cyan, BrightWhite, Engaged, Disengaged};
void add_str(WINDOW *w, const char *str, Color color = Color::Default, bool bold = false) {
  if (color != Color::Default) wattron(w, COLOR_PAIR(color));
  if (bold) wattron(w, A_BOLD);
  waddstr(w, str);
  if (bold) wattroff(w, A_BOLD);
  if (color != Color::Default) wattroff(w, COLOR_PAIR(color));
}
int main()
{
  initscr();	/* Start curses mode */
  clear();
  curs_set(false);  // not showing printf
  cbreak();  // Line buffering disabled. pass on everything
  noecho();
  intrflush(stdscr, FALSE);
  keypad(stdscr, true);  // not showing printf
  //nodelay(stdscr, true);  // not showing win
  start_color();
  init_pair(Color::Debug, 246, COLOR_BLACK);  // #949494
  //init_pair(Color::Yellow, 184, COLOR_BLACK);
  //init_pair(Color::Engaged, 28, 28);
  std::array<WINDOW*, Win::Max> w{};  // <ncurses.h>: WINDOW, Win::Max
  int max_width, max_height;
  getmaxyx(stdscr, max_height, max_width);
  printf("//--- max_height = %d, max_width = %d\n", max_height, max_width);
    //--- max_height = 11, max_height = 80
  w.fill(nullptr);
  w[Win::Title] = newwin(1, max_width, 0, 0);
  wbkgd(w[Win::Title], A_REVERSE);
  mvwprintw(w[Win::Title], max_height/2, 3, "openpilot replay %s", "0.9.1");  // why not showing??
  //mvprintw(max_height/2, 3, "openpilot replay %s", "0.9.1 ");  // OK showing
  wrefresh(w[Win::Title]);
  std::tuple<Color, const char *, bool> indicators[]{
      {Color::Engaged, " Engaged ", false},
      {Color::Disengaged, " Disengaged ", false},
      {Color::Green, " Info ", true},
      {Color::Yellow, " Warning ", true},
      {Color::Red, " Critical ", true},
      {Color::Cyan, " User Tag ", true},
  };
  //for (auto [color, name, bold] : indicators) {  // Error: auto
    add_str(w[Win::TimelineDesc], "__", Color::Green, true);
    add_str(w[Win::TimelineDesc], " Info ");
  //}
	printw("Hello World !!!");	/* Print Hello World */
	//refresh();	/* Print it on to the real screen */
	getch();	  /* Wait for user input */
	endwin();	  /* End curses mode */
	return 0;
}
CCCCCCCCCCCCCCCCCCCCCCCC
/*
C++ 15  W3  2023.3.3
for #define rInfo(fmt, ...) ::logMessage(ReplyMsgType::Info, fmt,  ## __VA_ARGS__)
    va_list, va_start, va_end
*/
#include<iostream>
#include<cstdarg>
using namespace std;
int sum (int n, ...)
{
    cout << "n = " << n << endl;
    va_list list;
    va_start (list, n);
    cout << list << endl;
    int x;
    int s = 0;
    for (int i = 0; i < n; i++)
    {
        x = va_arg (list, int);
        cout << "x = " << x << endl;
        s += x;
    }
    return s;
}
int main()
{
    cout << sum (3, 1, 2, 3) << " is sum" << endl;
    cout << sum (5, 1, 2, 3, 4, 5) << " is sum" << endl;
}
CCCCCCCCCCCCCCCCCCCCCCCC
/*
C++ 14  W3  2023.2.25
for #define rInfo(fmt, ...) ::logMessage(ReplyMsgType::Info, fmt,  ## __VA_ARGS__)
    va_list, va_start, va_end
from Standard alternative to GCC's ##__VA_ARGS__ trick?
     https://dotnettutorials.net/lesson/ellipsis-in-cpp/
*/
#include <string>
#define rInfo(fmt, ...) BAR(fmt "\n", ## __VA_ARGS__)
void BAR(const char* fmt, ...){
  va_list args;  // OK: <string> defines it?
  //va_start(args, fmt);  // NG: who defines it?
  printf(fmt, args);
  //va_end(args);  // NG: who defines it?
}
int main(){
  rInfo("Hello");
  rInfo("World!");
  return 0;
}
CCCCCCCCCCCCCCCCCCCCCCCC
/*
C++ 13  W3  2023.2.24
for replay_flags |= flag; scene.ignition |= pandaState.getIgnitionLine()
    route = args.empty() ? DEMO_ROUTE : args.first()
    for (const auto &[name, flag, _] : flags)
*/
#include <iostream>
#include <bitset>
#include <tuple>
using namespace std;
enum REPLAY_FLAGS {
  REPLAY_FLAG_NONE = 0x0000,
  REPLAY_FLAG_DCAM = 0x0002,
  REPLAY_FLAG_NO_FILE_CACHE = 0x0020,
};
int main() {
  int x = 5; int y = 3;
  bool ignition, ignition_on = true, ignition_off = false;
  std::string rt = "abc";
  std::string route = ignition_off ? "xyz" : rt;
  cout << "route = " << route << endl;
  uint32_t replay_flags = REPLAY_FLAG_NONE;
  uint32_t flag = REPLAY_FLAG_NO_FILE_CACHE;
  cout << flag << endl;
  std::string flagb = std::bitset<8>(flag).to_string();
  cout << "flagb = " << flagb << endl;
  cout << (replay_flags |= flag) << endl;
  cout << (replay_flags ^ flag) << endl;
  cout << (ignition |= ignition_on) << endl;
  std::string xb = std::bitset<8>(x).to_string();
  std::string yb = std::bitset<8>(y).to_string();
  cout << "xb = " << xb << endl;
  cout << "yb = " << yb << endl;
  cout << (x |= y) << endl;
  std::string xb1 = std::bitset<8>(x).to_string();
  cout << "xb1 = " << xb1 << endl;
  const std::tuple<std::string, REPLAY_FLAGS, std::string> flags[] = {
      {"dcam", REPLAY_FLAG_DCAM, "load driver camera"},
      {"no-cache", REPLAY_FLAG_NO_FILE_CACHE, "turn off local cache"},
  };
    // for-each loop: for (type variableName : arrayName), for (int i : array)
  for (auto &[name, _, desc] : flags) {
    cout << "name = " << name << endl;
    cout << "desc = " << desc << endl;
  }
  return 0;
}
/* output:
route = abc
32
flagb = 00100000
32
0
1
xb = 00000101
yb = 00000011
7
xb1 = 00000111*/
CCCCCCCCCCCCCCCCCCCCCCCC
/*
C++ 12  2023.2.22
from Qt Signals and Slots explained with Example Codes
  Define your own class with Qt signals and slots:
  Signals & Slots: https://doc.qt.io/qt-6/signalsandslots.html

for emit offroadTransition(!scene.started);
    QObject::connect

file name: ui/mainJLL.cc with mainJLL.h
ui/SConscript
  # build C++ 11  2023.2.22
  qt_src = ["mainJLL.cc"]
  qt_env.Program("_uiJLL", qt_src, LIBS=qt_libs)

. ~/sconsvenv/bin/activate
(sconsvenv) jinn@Liu:~/openpilot/$ scons
(sconsvenv) jinn@Liu:~/openpilot/selfdrive/ui/$ ./_uiJLL

back to
ui/SConscript
  # build uiJLL
got:
/usr/bin/ld: /tmp/scons_cache/moc_files/moc_mainJLL.cc:79: undefined reference to `JLLClass::setValue(int)'
scons: *** [selfdrive/ui/_uiJLL] Error 1
OK: renamed mainJLL.h to mainJLL2.h
*/
#include <QApplication>
#include <QMessageBox>
#include <QString>
#include "selfdrive/ui/mainJLL.h"
JLLClass::JLLClass() {
  val = 0;
}
void JLLClass::setValue( int v ) {
  if ( v != val ) {
    val = v;
      //emit valueChanged(v);  // emits new v of val
    emit valueChanged(val);  // OK: same
      // the identifier 'emit' says that this is a signal (not function) call
      // valueChanged() is a Qt signal function
      //valueChanged(val);  // OK: same without 'emit'
  }
}
  // simple message box
void MyMessageBox(JLLClass& a, JLLClass& b, QString title) {
  QString Qstr, Qval;
    // Tinker the string together
  Qstr.append(title);
  Qstr.append("\na: "); Qval.setNum(a.value()); Qstr.append(Qval);
  Qstr.append("\nb: "); Qval.setNum(b.value()); Qstr.append(Qval);
  QMessageBox::information(NULL, "JLLClass Information", Qstr, QMessageBox::Ok);
}
int main(int argc, char *argv[]) {
  QApplication app(argc, argv);
    // Two JLLClass objects
  JLLClass *a = new JLLClass();
  JLLClass *b = new JLLClass();
  QObject::connect( a, SIGNAL( valueChanged(int) ), b, SLOT( setValue(int) ) );
    // The signal of object a is sent to the slot object b connected a.
    // The other way to connect a signal to a slot is to use QObject::connect()
    //   and the SIGNAL and SLOT macros.
    // A signal is emitted (SIGNAL()) when a particular event occurs (valueChanged).
    // A slot function (SLOT()) is called in response (setValue) to a particular signal.
    // setValue() is a Qt or JLL slot function ??? Qt I guess.
  b->setValue( 100 );  // b.val gets the value 100
  MyMessageBox(*a, *b, "b->setValue(100)");
    // a->setValue sets 99 and sends SIGNAL( valueChanged(99) ) to b's
    // SLOT( setValue(99) ) because a and b are connected.
    // This process is achieved by clicking the button "OK" in QMessageBox.
  a->setValue( 99 );
    // The proof
  MyMessageBox(*a, *b, "a->setValue(99)");
  return 0;
}
CCCCCCCCCCCCCCCCCCCCCCCC
/*
C++ 11  2023.2.22
for ui/mainJLL.cc
file name: ui/mainJLL.h
*/
#include <QObject>
  // A class that has signals and slots.
class JLLClass: public QObject {
  Q_OBJECT
public:
  JLLClass();
  int value() const { return val; }
public slots:  // results: (a: 99, b: 99)
//public:
  // No "slots" yields different (a: 99, b: 100) and the warning:
  // QObject::connect: No such slot JLLClass::setValue(int)
    // The value of "val" is changed.
  void setValue(int);
signals:
    // The signal should be sent out ("emit"), when "val" is changed.
  void valueChanged(int);  // written by Meta Object Compiler (MOC)
private:
  int val;
}; // put ;
CCCCCCCCCCCCCCCCCCCCCCCC
/*
C++ 10  W3  2023.2.22
from Lambda expression in C++
  [&] : capture all external variables by reference
  [=] : capture all external variables by value
  [a, &b] : capture a by value and b by reference
  [ ] can only access variables which are local to it.
for QObject::connect(alert_notif, &QPushButton::clicked, [=] { center_layout->setCurrentIndex(2); });
    connect(1,2,3,4) or connect(1,2,3)
    The widget() function returns the widget at a given index position.
    The index of the widget that is shown on screen is given by currentIndex()
      and can be changed using setCurrentIndex().
*/
#include <bits/stdc++.h>
using namespace std;
void printVector(vector<int> v)
{
  // lambda expression to print vector
  for_each(v.begin(), v.end(), [](int i) {
    std::cout << i << " ";
  });
  cout << endl;
}
int main()
{
  vector<int> v1 = {3, 1, 7, 9};
  vector<int> v2 = {1, 2, 7, 16};
    //  access v1 and v2 by reference
    //auto pushinto = [=] (int m) // error
    //auto pushinto = [] (int m)  // error: ‘v1’ is not captured
  auto pushinto = [&] (int m) {
    v1.push_back(m);
    v2;
  };
    // it pushes 20 in both v1 and v2
  pushinto(20);
  printVector(v1);
  printVector(v2);
  int N = 5;
    // [N] can access only N by value
  vector<int>:: iterator p = find_if(v1.begin(), v1.end(), [N](int i) {
    //N = N + 1; // error
    return i > N;
  });
  cout << "First number greater than 5 is : " << *p << endl;
    // function to count numbers greater than or equal to N
    // [=] can access all variable
  int count_N = count_if(v1.begin(), v1.end(), [=](int a) {
    //N = N + 1; // error
    return (a >= N);
  });
  cout << "The number of elements greater than or equal to 5 is : "
     << count_N << endl;
}
CCCCCCCCCCCCCCCCCCCCCCCC
return (x > threshold).type(x.dtype)
??? typedef void (*sighandler_t)(int);
??? sighandler_t signal(int sig, sighandler_t func);
  - signal is a function taking two parameters:
    (an int) and (a pointer to a function taking an int and returning void),
    and returning a pointer to a (function taking an int and returning void)
  - The typedef declares a pointer to a function (taking an int parameter and
    returning nothing). The function signal can now be seen as a function
    that takes two parameters (an int and a pointer to a function)
    and returns a pointer to a function.
CCCCCCCCCCCCCCCCCCCCCCCC
/*
C++ 9   2022.1.4
C library function - signal()
??? signal(SIGINT, (sighandler_t)set_do_exit);
  in selfdrive/modeld/modeld.cc
*/
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
void sighandler(int);
int main () {
   signal(SIGINT, sighandler);

   while(1) {
      printf("Going to sleep for a second...\n");
      sleep(1);
   }
   return(0);
}
void sighandler(int signum) {
   printf("Caught signal %d, coming out...\n", signum);
   exit(1);
}
CCCCCCCCCCCCCCCCCCCCCCCC
/*
C++ 8   2021.5.8
??? ::capnp::MallocMessageBuilder message;
    in https://capnproto.org/cxx.html#kj-library
*/
// https://stackoverflow.com/questions/15649580/using-in-c
// :: is known as the scope resolution operator.
#include <iostream>
using namespace std;
const int x = 5;
namespace foo {
  const int x = 0;
}
int bar() {
  int x = 1;
  return x;
}
struct Meh {
  static const int x = 2;
};
int main() {
  std::cout << x; // => 5
  {
    int x = 4;
    std::cout << x; // => 4
    std::cout << ::x; // => 5, ::x looks for x outside the current scope
  }
  std::cout << Meh::x; // => 2, use the definition of x inside the scope of Meh
  std::cout << foo::x; // => 0, use the definition of x inside foo
  std::cout << bar() << endl; // => 1, use the definition of x inside bar (returned by bar)
}
CCCCCCCCCCCCCCCCCCCCCCCC
/*
C++ 7   2021.4.27
Big-Endian 與 Little-Endian 的差異與判斷程式碼
C 語言判斷位元組順序
*/
#include <stdio.h>
typedef union {
  unsigned long l;
  unsigned char c[4];
} EndianTest;
int main() {
  EndianTest et;
  et.l = 0x12345678;
  printf("本系統位元組順序為：");
  if (et.c[0] == 0x78 && et.c[1] == 0x56 && et.c[2] == 0x34 && et.c[3] == 0x12) {
    printf("Little Endiann");
  } else if (et.c[0] == 0x12 && et.c[1] == 0x34 && et.c[2] == 0x56 && et.c[3] == 0x78) {
    printf("Big Endiann");
  } else {
    printf("Unknown Endiann");
  }
  printf("0x%lX 在記憶體中的儲存順序：n", et.l);
  for (int i = 0; i < 4; i++) {
    printf("%p : 0x%02Xn", &et.c[i], et.c[i]);
  }
  return 0;
}
CCCCCCCCCCCCCCCCCCCCCCCC
/*
C++ 6   2021.4.26
Cap'n Proto (C++ Developer Meetup Iasi)
https://capnproto.org/cxx.html
(OP) jinn@Liu:~/openpilot/acpp$ g++ z.cpp -std=c++11 -o z

??? /home/jinn/can/zc1.py/CAN 10: cereal::ModelDataV2, kj::ArrayPtr<const float> raw_pred
??? /home/jinn/openpilot/acpp/panda.h: //cereal::PandaState
  --- more on Cap’n Proto
  Using Cap’n Proto in Rust to serialize and deserialize objects
  capnp 0.2.1 documentation
  - Calling C++ code from Python tends to be painful or slow.
    With Cap’n Proto, the two languages can easily operate on
    the same in-memory data structure.
  - https://www.slideshare.net/ovidiuf/capn-proto-c-developer-meetup-iasi
    Serialization Uses a schema file (AddressBook.capnp) to define the message structure.
    capnp compile -oc++ AddressBook.capnp => AddressBook.capnp.h, AddressBook.capnp.c++
  https://capnp.readthedocs.io/en/latest/quickstart.html
  https://capnproto.org/cxx.html
  https://capnproto.org/cxxrpc.html
  https://stackoverflow.com/questions/38285733/how-to-get-byte-into-capnpdata
  https://capnproto.github.io/pycapnp/
*/
#include <iostream>
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include "log.capnp.h"
using namespace std;
/**/
int main ()
{
  cout << "$$$$$ data = " << endl;
}
CCCCCCCCCCCCCCCCCCCCCCCC
/*
C++ 5   2021.4.25
??? openpilot/selfdrive/boardd/panda.cc
  (OP) jinn@Liu:~/openpilot/acpp$ g++ z.cpp panda.cc log.capnp.c++ car.capnp.c++ -std=c++11 -o z
  /home/jinn/openpilot/acpp/log.capnp.c++
  /home/jinn/openpilot/acpp/car.capnp.c++
  (OP) jinn@Liu:~/openpilot/acpp$ g++ z.cpp panda.cc -std=c++11 -o z
  /home/jinn/openpilot/acpp/panda.cc
  /home/jinn/openpilot/acpp/panda.h
*/
#include <iostream>
#include "panda.h"
using namespace std;
/**/
int main ()
{
  Panda * panda = nullptr;
  uint32_t data[RECV_SIZE/4];
  cout << "$$$$$ data = " << data << endl;
  cout << "$$$$$ RECV_SIZE = " << RECV_SIZE << endl;
  int recv = panda -> usb_bulk_read(0x81, (unsigned char*)data, RECV_SIZE, TIMEOUT);
  cout << "$$$$$ recv = " << recv << endl;
}
CCCCCCCCCCCCCCCCCCCCCCCC
/*
C++ 4   2021.4.24
in zc1.py CAN 10   2021.4.21-4.24
??? send_raw_pred
  openpilot/selfdrive/modeld/modeld.cc
  openpilot/selfdrive/modeld/models/commonmodel.h
  const bool send_raw_pred = getenv("SEND_RAW_PRED") != NULL;
  char* getenv (const char* name);
*/
// http://www.cplusplus.com/reference/cstdlib/getenv/
/* getenv example: getting path */
#include <stdio.h>      /* printf */
#include <stdlib.h>     /* getenv */
#include <iostream>
using namespace std;
int main ()
{
  char* pPath;
  pPath = getenv ("PATH");
  if (pPath!=NULL)
    printf ("The current path is: %s",pPath);

  cout << endl;
  const bool send_raw_pred = getenv("SEND_RAW_PRED") != NULL;
  cout << "send_raw_pred = " << send_raw_pred << endl;
}
CCCCCCCCCCCCCCCCCCCCCCCC
/*
C++ 3,  JLL 2021.4.22
??? data()
  openpilot/selfdrive/modeld/modeld.cc
  ModelState model;
  model_publish(pm, ..., model_buf, ..., model.output.data(), ...);
  phonelibs/acado/include/acado/external_packages/eigen3/Eigen/src/Core/MapBase.h
  inline const Scalar* data() const { return m_data; }
*/
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <eigen3/Eigen/Dense>
using namespace Eigen;
using namespace std;
int main()
{
  typedef typename internal::traits<Derived>::Scalar Scalar;
  inline const Scalar* data() const { return m_data; }
}
CCCCCCCCCCCCCCCCCCCCCCCC
/*
C++ 2,  JLL 2021.4.22
https://eigen.tuxfamily.org/dox/GettingStarted.html
*/
#include <iostream>
#include <eigen3/Eigen/Dense>
using namespace Eigen;
using namespace std;
int main()
{
  MatrixXd m(2,2);
  VectorXd v(2);
  m(0,0) = 3;
  m(1,0) = 2.5;
  m(0,1) = -1;
  m(1,1) = m(1,0) + m(0,1);
  cout << m << endl;

  v << 1, 2;
  cout << v << endl;
  cout << "m * v =" << endl << m * v << endl;
  Matrix3f A;
  Vector3f b;
  A << 1,2,3,  4,5,6,  7,8,10;
  b << 3, 3, 4;
  cout << "Here is the matrix A:\n" << A << endl;
  cout << "Here is the vector b:\n" << b << endl;
  Vector3f x = A.colPivHouseholderQr().solve(b);
  cout << "The solution is:\n" << x << endl;

  Vector3d v(1,2,3);
  Vector3d w(0,1,2);
  cout << "Dot product: " << v.dot(w) << endl;
  double dp = v.adjoint()*w; // automatic conversion of the inner product to a scalar
  cout << "Dot product via a matrix product: " << dp << endl;
  cout << "Cross product:\n" << v.cross(w) << endl;
}
CCCCCCCCCCCCCCCCCCCCCCCC
/*
C++ 1,  JLL 2021.4.6, 2022.1.5
??? openpilot/selfdrive/modeld/models/driving.cc
  jinn@Liu:~/OP079C2/selfdrive/modeld/test/polyfit$ g++ mainJL.cc -o mainJL
  jinn@Liu:~/OP079C2/selfdrive/modeld/test/polyfit$ ./mainJL
  [6.65505e-07,-0.000212898,-0.00963532,3.26513]
*/
