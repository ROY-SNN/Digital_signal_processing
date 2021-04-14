% =========================================================================
% =========================================================================
% ����Ҫ˵������
% �����źŴ���ʵ�飺�����˲���
% �༶������182��
% С���Ա�������ɡ�������
% 
% ��ʵ�ֹ��ܡ���
% 1.�ɼ������ԭʼ��Ƶ�źţ������Ƴ���ʱ����ͼ��Ƶ��ͼ��
% 2.Ϊԭʼ�ź��������(2��)�� a.4KHz�����Ҳ�  b.������ɵİ�����
% 3.ʵ�����ֵ�ͨ�˲������������������������������λFIR�����˲���
%                      ��һ��Ƶ��=2*fc/fs
% 4.Ϊ�����������źŽ����˲������ָ���ԭʼ�ź�
% 5.�����źű仯���̣������Բ��ų�����
% =========================================================================
% =========================================================================
% ��ʼ��
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
% ��lys_xml�ɼ�֮ǰִ��
function lys_xml_OpeningFcn(hObject, eventdata, handles, varargin)
handles.output = hObject;
guidata(hObject, handles);
% =========================================================================
function varargout = lys_xml_OutputFcn(hObject, eventdata, handles) 
varargout{1} = handles.output;
% =========================================================================
% �����屳�����á�
% =========================================================================
function figure1_CreateFcn(hObject, eventdata, handles)
ha=axes('units','normalized','pos',[0 0 1 1]); 
uistack(ha,'down');
set(ha,'handlevisibility','off','visible','off');
% =========================================================================
% ��������ԭʼ�źš�����ԭʼ�źŵ�ʱ����ͼ+Ƶ��ͼ
% =========================================================================
function pushbutton1_Callback(hObject, eventdata, handles)
global y;
global fs;
global n;
global y1;
global y2;
global f;
y1 = subplot(2,1,1);
plot(y);  title("ԭʼ��Ƶ�źŵ�ʱ����");  xlabel("ʱ��");
ylim([-1 1])
y2 = subplot(2,1,2);
% ѡȡ�任�ĵ���
n = length(y);
% ��n����и���Ҷ�任��Ƶ��
y_p = fft(y,n);
% ��Ӧ���Ƶ��
f = fs*(0:n/2-1)/n;
plot(f,abs(y_p(1:n/2)));title("ԭʼ��Ƶ�źŲ������Ƶ��ͼ");xlabel("Ƶ��(Hz)");
xlim([0 1000])
% =========================================================================
% ��ѡ�����1������ԭʼ��Ƶ�ź�
% =========================================================================
function radiobutton2_Callback(hObject, eventdata, handles)
% audioread()����ֵy������������   fs����������
global y;
global fs;
% [y,fs]=audioread('C:\Users\LYS\Desktop\Audio-Digital-Processing-master\���������ļ�.wav');
[y,fs]=audioread('C:\Users\LYS\Desktop\���������ļ�2.mp3');
% ���������ź�
sound(y,fs);
% =========================================================================
% ������������1(4KHz���Ҳ�)������4KHz�����Ҳ������Ƶ�źţ�ʱ����ͼ+Ƶ��ͼ
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
title("����4KHz�����Ҳ�����ź�ʱ����"); xlabel("ʱ��"); ylabel("��ֵ");
y_szp = fft(y_sz,n);
f = fs*(0:n/2-1)/n;
y8 = subplot(2,1,2);
plot(f,abs(y_szp(1:n/2)))
title("����4KHz�����Ҳ�����ź�Ƶ��ͼ"); xlabel("Ƶ��(Hz)"); ylabel("��ֵ")
xlim([0 5000])
ylim([0 4000])
% =========================================================================
% ��ѡ�����2������4KHz�����Ҳ������Ƶ�ź�
% =========================================================================
function radiobutton5_Callback(hObject, eventdata, handles)
global y;
global fs;
global y_sz;
L = length(y);
t=linspace(0,(L-1)/fs,L);
% ����4KHz�����Ҳ�����
s_noise = 0.5*sin(2*pi*4000*t);
s_noise = s_noise';
% �źŵ���(��4KHz�����Ҳ�)
y_sz = y+s_noise;
sound(y_sz,fs);
% =========================================================================
% ������������2(���������)������������������Ƶ�źţ�ʱ����ͼ+Ƶ��ͼ
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
title("���������������ź�ʱ����");
xlabel("ʱ��(s)");
y_zp = fft(y_z,n);
f = fs*(0:n/2-1)/n;
y4 = subplot(2,1,2);
xlim([0 1000])
plot(f,abs(y_zp(1:n/2)));
title("���������������ź�Ƶ��ͼ");
xlabel("Ƶ��(Hz)");
xlim([0 1000])
% =========================================================================
% ��ѡ�����3�����ŵ�����������������Ƶ�ź�
% =========================================================================
function radiobutton3_Callback(hObject, eventdata, handles)
global y;
global fs;
global y_z;
% ������Ƶ�źŵĳ���
L = length(y);
% �����ȳ��ȵ�����ź�(������Сȡ������������ķ��ȱ���)
% randn(m,n)������m*n���������(���Ӿ�ֵ0,����1��׼��̬�ֲ�)
% ������(������ǰ�������������
noise = 0.1*randn(L,1);
% �źŵ���(����������)
y_z = y+noise;
sound(y_z,fs);
% =========================================================================
% ����������ͨ�˲�������������������Ƶ�źţ�ʱ����ͼ+Ƶ��ͼ            
%����ע������������(����)���������λFIR�����˲���
% ��ֹƵ��fc=1700KHz�������С˥����100
% ����a=0.112*(�����С˥��-8.7)
% �˲�������N=�������С˥��-8��/2.285���ɴ����
% ���ɴ����wde1=|ws-wp|
% =========================================================================
function pushbutton3_Callback(hObject, eventdata, handles)
global fs;
global b;
% ��ʼ������ͨ�˲���������ָ��
fp = 1500;    %800;%1500;
fc = 1700;    %����Ƶ�ʣ�1000;%1700;
As = 100;     %�����С˥����20��100      
Ap = 1;
fprintf('the value of ����Ƶ��fs %6.4f\n', fs);
fprintf('the value of ��ֹƵ��fc %6.4f\n', fc);
wc = 2*pi*fc/fs;             fprintf('the value of wc %6.4f\n', wc);
wp = 2*pi*fp/fs;             fprintf('the value of wp %6.4f\n', wp);
wdel = wc-wp;                fprintf('the value of ���ɴ�����wdel %6.4f\n', wdel);
beta = 0.112*(As-8.7);       fprintf('the value of ����beta %6.4f\n', beta);
% ceil���������������ȡ��
N = ceil((As-8)/2.285/wdel); fprintf('the value of �˲�������N %6.2f\n', N);
wn = kaiser(N+1,beta); % wn��һ����
ws = (wp+wc)/2/pi;           fprintf('the value of ��һ��Ƶ��ws %6.4f\n', ws);
% �����������������λFIR�����˲���
b = fir1(N,ws,'low',wn);
figure(1)
freqz(b,1);
% =========================================================================
% ����������ͨ�˲����������������Ƶ�źţ�ʱ����ͼ+Ƶ��ͼ
%����ע������������(����)���������λFIR�����˲���
% ��ֹƵ��fc=1700KHz�������С˥����100
% ����a=0.112*(�����С˥��-8.7)
% �˲�������N=�������С˥��-8��/2.285���ɴ����
% ���ɴ����wde1=|ws-wp|
% =========================================================================
function pushbutton6_Callback(hObject, eventdata, handles)
global fs;  
global bs;
% ��ʼ������ͨ�˲���������ָ��
fp = 1700;
fc = 4410;
As = 100;
Ap = 1;
fprintf('the value of ����Ƶ��fs %6.4f\n', fs);
fprintf('the value of ��ֹƵ��fc %6.4f\n', fc);
wc = 2*pi*fc/fs;            fprintf('the value of wc %6.4f\n', wc);
wp = 2*pi*fp/fs;            fprintf('the value of wp %6.4f\n', wp); 
wdel = wc-wp;               fprintf('the value of ���ɴ�����wdel %6.4f\n', wdel);
beta = 0.112*(As-8.7);      fprintf('the value of ����beta %6.4f\n', beta);
% ceil���������������ȡ��
N = ceil((As-8)/2.285/wdel);fprintf('the value of �˲�������N %6.2f\n', N);
wn = kaiser(N+1,beta);        
ws = (wp+wc)/2/pi;          fprintf('the value of ��һ��Ƶ��ws %6.4f\n', ws);  
% �����������������λFIR�����˲���
bs = fir1(N,ws,'low',wn);
figure(2)
freqz(bs,1);
% =========================================================================
% ���������˲�����ź�1������4KHz���Ҳ����źž�����ͨ�˲�����ʱ����ͼ+Ƶ��ͼ
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
title("����4KHz���Ҳ����ź��˲���Ĳ���")
y10 = subplot(2,1,2);
plot(f,abs(MS(1:n/2)));
xlim([0 5000]);
title("����4KHz���Ҳ����ź��˲����Ƶ��")
% =========================================================================
% ��ѡ�����4�������ŵ���4KHz���Ҳ����źž�����ͨ�˲�������ź�
% =========================================================================
function radiobutton6_Callback(hObject, eventdata, handles)
global ms;
global fs;
sound(ms,fs)
% =========================================================================
% ���������˲�����ź�2����������������źž�����ͨ�˲�����ʱ����ͼ+Ƶ��ͼ
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
title("��������������ź��˲���Ĳ���")
y6 = subplot(2,1,2);
plot(f,abs(M(1:n/2)));
xlim([0 1000]);
title("��������������ź��˲����Ƶ��")
% =========================================================================
% ��ѡ�����5�������ŵ�������������źž�����ͨ�˲������ź�
% =========================================================================
function radiobutton4_Callback(hObject, eventdata, handles)
global m;
global fs;
sound(m,fs)
