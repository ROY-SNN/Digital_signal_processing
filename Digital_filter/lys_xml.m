% =========================================================================
% =========================================================================
% 【简要说明】：
% 数字信号处理实验：数字滤波器
% 班级：电子182班
% 小组成员：李延松、熊梦龙
% 
% 【实现功能】：
% 1.采集输入的原始音频信号，并绘制出：时域波形图、频谱图像
% 2.为原始信号添加噪音(2种)： a.4KHz的正弦波  b.随机生成的白噪音
% 3.实现数字低通滤波器：窗函数法（凯塞）设计线性相位FIR数字滤波器
%                      归一化频率=2*fc/fs
% 4.为含有噪音的信号进行滤波处理，恢复到原始信号
% 5.整个信号变化过程，均可以播放出声音
% =========================================================================
% =========================================================================
% 初始化
% =========================================================================
function varargout = lys_xml(varargin)
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @lys_xml_OpeningFcn, ...
                   'gui_OutputFcn',  @lys_xml_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% =========================================================================
% 在lys_xml可见之前执行
function lys_xml_OpeningFcn(hObject, eventdata, handles, varargin)
handles.output = hObject;
guidata(hObject, handles);
% =========================================================================
function varargout = lys_xml_OutputFcn(hObject, eventdata, handles) 
varargout{1} = handles.output;
% =========================================================================
% 【窗体背景设置】
% =========================================================================
function figure1_CreateFcn(hObject, eventdata, handles)
ha=axes('units','normalized','pos',[0 0 1 1]); 
uistack(ha,'down');
set(ha,'handlevisibility','off','visible','off');
% =========================================================================
% 【按键：原始信号】绘制原始信号的时域波形图+频谱图
% =========================================================================
function pushbutton1_Callback(hObject, eventdata, handles)
global y;
global fs;
global n;
global y1;
global y2;
global f;
y1 = subplot(2,1,1);
plot(y);  title("原始音频信号的时域波形");  xlabel("时间");
ylim([-1 1])
y2 = subplot(2,1,2);
% 选取变换的点数
n = length(y);
% 对n点进行傅里叶变换到频域
y_p = fft(y,n);
% 对应点的频率
f = fs*(0:n/2-1)/n;
plot(f,abs(y_p(1:n/2)));title("原始音频信号采样后的频谱图");xlabel("频率(Hz)");
xlim([0 1000])
% =========================================================================
% 【选项：播放1】播放原始音频信号
% =========================================================================
function radiobutton2_Callback(hObject, eventdata, handles)
% audioread()返回值y——样本数据   fs——采样率
global y;
global fs;
% [y,fs]=audioread('C:\Users\LYS\Desktop\Audio-Digital-Processing-master\声音测试文件.wav');
[y,fs]=audioread('C:\Users\LYS\Desktop\声音测试文件2.mp3');
% 播放语音信号
sound(y,fs);
% =========================================================================
% 【按键：噪声1(4KHz正弦波)】叠加4KHz的正弦波后的音频信号：时域波形图+频谱图
% =========================================================================
function pushbutton5_Callback(hObject, eventdata, handles)
global y;
global fs;
global y7;
global y8;
global y_sz;
y7 = subplot(2,1,1);
plot(y_sz);
n = length(y);
title("叠加4KHz的正弦波后的信号时域波形"); xlabel("时间"); ylabel("幅值");
y_szp = fft(y_sz,n);
f = fs*(0:n/2-1)/n;
y8 = subplot(2,1,2);
plot(f,abs(y_szp(1:n/2)))
title("叠加4KHz的正弦波后的信号频谱图"); xlabel("频率(Hz)"); ylabel("幅值")
xlim([0 5000])
ylim([0 4000])
% =========================================================================
% 【选项：播放2】叠加4KHz的正弦波后的音频信号
% =========================================================================
function radiobutton5_Callback(hObject, eventdata, handles)
global y;
global fs;
global y_sz;
L = length(y);
t=linspace(0,(L-1)/fs,L);
% 生成4KHz的正弦波噪声
s_noise = 0.5*sin(2*pi*4000*t);
s_noise = s_noise';
% 信号叠加(加4KHz的正弦波)
y_sz = y+s_noise;
sound(y_sz,fs);
% =========================================================================
% 【按键：噪声2(随机白噪声)】叠加随机噪声后的音频信号：时域波形图+频谱图
% =========================================================================
function pushbutton2_Callback(hObject, eventdata, handles)
global y;
global y_z;
global fs;
global y3;
global y4;
global y_zp;
y3 = subplot(2,1,1);
plot(y_z);
n = length(y);
title("叠加随机噪声后的信号时域波形");
xlabel("时间(s)");
y_zp = fft(y_z,n);
f = fs*(0:n/2-1)/n;
y4 = subplot(2,1,2);
xlim([0 1000])
plot(f,abs(y_zp(1:n/2)));
title("叠加随机噪声后的信号频谱图");
xlabel("频率(Hz)");
xlim([0 1000])
% =========================================================================
% 【选项：播放3】播放叠加随机白噪声后的音频信号
% =========================================================================
function radiobutton3_Callback(hObject, eventdata, handles)
global y;
global fs;
global y_z;
% 计算音频信号的长度
L = length(y);
% 产生等长度的随机信号(噪声大小取决于随机函数的幅度倍数)
% randn(m,n)：返回m*n的随机矩阵(服从均值0,方差1标准正态分布)
% 白噪声(方差就是白噪声的能量）
noise = 0.1*randn(L,1);
% 信号叠加(加噪声处理)
y_z = y+noise;
sound(y_z,fs);
% =========================================================================
% 【按键：低通滤波器】加随机噪音后的音频信号：时域波形图+频谱图            
%【备注】：窗函数法(凯塞)设计线性相位FIR数字滤波器
% 截止频率fc=1700KHz、阻带最小衰减：100
% 参数a=0.112*(阻带最小衰减-8.7)
% 滤波器阶数N=（阻带最小衰减-8）/2.285过渡带宽度
% 过渡带宽度wde1=|ws-wp|
% =========================================================================
function pushbutton3_Callback(hObject, eventdata, handles)
global fs;
global b;
% 初始化：低通滤波器的性能指标
fp = 1500;    %800;%1500;
fc = 1700;    %截至频率：1000;%1700;
As = 100;     %阻带最小衰减：20、100      
Ap = 1;
fprintf('the value of 采样频率fs %6.4f\n', fs);
fprintf('the value of 截止频率fc %6.4f\n', fc);
wc = 2*pi*fc/fs;             fprintf('the value of wc %6.4f\n', wc);
wp = 2*pi*fp/fs;             fprintf('the value of wp %6.4f\n', wp);
wdel = wc-wp;                fprintf('the value of 过渡带带宽wdel %6.4f\n', wdel);
beta = 0.112*(As-8.7);       fprintf('the value of 参数beta %6.4f\n', beta);
% ceil函数：朝正无穷方向取整
N = ceil((As-8)/2.285/wdel); fprintf('the value of 滤波器阶数N %6.2f\n', N);
wn = kaiser(N+1,beta); % wn是一组数
ws = (wp+wc)/2/pi;           fprintf('the value of 归一化频率ws %6.4f\n', ws);
% 窗函数法设计线性相位FIR数字滤波器
b = fir1(N,ws,'low',wn);
figure(1)
freqz(b,1);
% =========================================================================
% 【按键：低通滤波器】加噪音后的音频信号：时域波形图+频谱图
%【备注】：窗函数法(凯塞)设计线性相位FIR数字滤波器
% 截止频率fc=1700KHz、阻带最小衰减：100
% 参数a=0.112*(阻带最小衰减-8.7)
% 滤波器阶数N=（阻带最小衰减-8）/2.285过渡带宽度
% 过渡带宽度wde1=|ws-wp|
% =========================================================================
function pushbutton6_Callback(hObject, eventdata, handles)
global fs;  
global bs;
% 初始化：低通滤波器的性能指标
fp = 1700;
fc = 4410;
As = 100;
Ap = 1;
fprintf('the value of 采样频率fs %6.4f\n', fs);
fprintf('the value of 截止频率fc %6.4f\n', fc);
wc = 2*pi*fc/fs;            fprintf('the value of wc %6.4f\n', wc);
wp = 2*pi*fp/fs;            fprintf('the value of wp %6.4f\n', wp); 
wdel = wc-wp;               fprintf('the value of 过渡带带宽wdel %6.4f\n', wdel);
beta = 0.112*(As-8.7);      fprintf('the value of 参数beta %6.4f\n', beta);
% ceil函数：朝正无穷方向取整
N = ceil((As-8)/2.285/wdel);fprintf('the value of 滤波器阶数N %6.2f\n', N);
wn = kaiser(N+1,beta);        
ws = (wp+wc)/2/pi;          fprintf('the value of 归一化频率ws %6.4f\n', ws);  
% 窗函数法设计线性相位FIR数字滤波器
bs = fir1(N,ws,'low',wn);
figure(2)
freqz(bs,1);
% =========================================================================
% 【按键：滤波后的信号1】叠加4KHz正弦波的信号经过低通滤波器：时域波形图+频谱图
% =========================================================================
function pushbutton8_Callback(hObject, eventdata, handles)
global y9;
global y10;
global bs;
global y_sz;
global n;
global ms;
global y;
global fs;
n = length(y);
f = fs*(0:n/2-1)/n;
ms = fftfilt(bs,y_sz);
MS = fft(ms,n);
y9 = subplot(2,1,1);
plot(ms);
xlim([0 450000]);
ylim([-1 1]);
title("叠加4KHz正弦波的信号滤波后的波形")
y10 = subplot(2,1,2);
plot(f,abs(MS(1:n/2)));
xlim([0 5000]);
title("叠加4KHz正弦波的信号滤波后的频谱")
% =========================================================================
% 【选项：播放4】：播放叠加4KHz正弦波的信号经过低通滤波器后的信号
% =========================================================================
function radiobutton6_Callback(hObject, eventdata, handles)
global ms;
global fs;
sound(ms,fs)
% =========================================================================
% 【按键：滤波后的信号2】叠加随机噪声的信号经过低通滤波器：时域波形图+频谱图
% =========================================================================
function pushbutton4_Callback(hObject, eventdata, handles)
global y5;
global y6;
global b;
global y_z;
global n;
global m;
global y;
global fs;
n = length(y);
f = fs*(0:n/2-1)/n;
m = fftfilt(b,y_z);
M = fft(m,n);
y5 = subplot(2,1,1);
plot(m);
xlim([0 450000]);
ylim([-1 1]);
title("叠加随机噪声的信号滤波后的波形")
y6 = subplot(2,1,2);
plot(f,abs(M(1:n/2)));
xlim([0 1000]);
title("叠加随机噪声的信号滤波后的频谱")
% =========================================================================
% 【选项：播放5】：播放叠加随机噪声的信号经过低通滤波器的信号
% =========================================================================
function radiobutton4_Callback(hObject, eventdata, handles)
global m;
global fs;
sound(m,fs)
