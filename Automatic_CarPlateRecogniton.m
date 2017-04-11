clc;    % Clear command window.
clear all;  % Delete all variables.
close all;  % Close all figure windows except those created by imtool.
imtool close all;   % Close all figure windows created by imtool.

a1= imread('D:\BE Projects\Number Plate Detection\SCW_MyCarLocal\21.jpg');

%filename = uigetfile('*.*');
%a1=imread(filename);

figure(1), imshow(a1),title('Original Image');
I=rgb2gray(a1);
figure(2), imshow(I), title('Original Image to Grayscale Image');

tic

[height,width]=size(I);

BW1 = edge(I, 'sobel', 'horizontal');
figure(3), imshow(BW1), title('Result of Horizontal Edges');

BW2 = edge(I, 'sobel', 'vertical');
figure(4), imshow(BW2), title('Result of Vertical Edges');

BW3 = BW1 | BW2;
figure(5), imshow(BW3), title('Result of OR Masking Operation');
% BW4 = ~BW3;
 %figure(6), imshow(BW4), title('Inverse of OR Masking Operation Output');

se=[1;1;1];
I3=imerode(BW3,se);
figure(6), imshow(I3);
%NHOOD=floor((size(I3)+1)/2)
%se=strel('arbitrary', NHOOD)
  
se=strel('rectangle',[20,20]);
I4=imclose(I3,se);
figure(7), imshow(I4);

IM3=bwareaopen(I4,30);
figure(8), imshow(IM3);


% Horizontal Projection
p_h=projection(double(IM3),'h');               
if(p_h(1)>0)
    p_h=[0,p_h];
end

%Vertical Projection
p_v=projection(double(IM3),'v');               
if(p_v(1)>0)
    p_v=[0,p_v];
end


p_h=double((p_h>5));
p_h=find(((p_h(1:end-1)-p_h(2:end))~=0));
len_h=length(p_h)/2;
%%%%%
p_v=double((p_v>5));
p_v=find(((p_v(1:end-1)-p_v(2:end))~=0));
len_v=length(p_v)/2;

% Find the objects
k=1;
for i=1:len_h
    for j=1:len_v
        s=IM3(p_h(2*i-1):p_h(2*i),p_v(2*j-1):p_v(2*j));
        if(mean(mean(s))>0.1)
            p{k}=[p_h(2*i-1),p_h(2*i)+1,p_v(2*j-1),p_v(2*j)+1];
            k=k+1;
        end
    end
end
k=k-1;

 if (k==0)
    disp('No License Plate Found');
 end

%Plot the edge and find the center and ratio
for i=1:k
   edge_IM3=double(edge(double(IM3(p{i}(1):p{i}(2),p{i}(3):p{i}(4))),'canny'));

   [x,y]=find(edge_IM3==1);
   p{i}=[p{i}(1)+min(x),p{i}(2)-(p{i}(2)-p{i}(1)+1-max(x)),...
         p{i}(3)+min(y),p{i}(4)-(p{i}(4)-p{i}(3)+1-max(y))];
   p_center{i}=[fix((p{i}(1)+p{i}(2))/2),fix((p{i}(3)+p{i}(4))/2)];
   p_ratio(i)=(p{i}(4)-p{i}(3))/(p{i}(2)-p{i}(1));
end

if k>1
    n=0;
    ncount=zeros(1,k);
    for i=1:k-1
       
        if(abs(p{i}(1)+p{i}(2)-p{i+1}(1)-p{i+1}(2))<=height/30&&abs(p{i+1}(3)-p{i}(4))<=width/15)
            p{i+1}(1)=min(p{i}(1),p{i+1}(1));
            p{i+1}(2)=max(p{i}(2),p{i+1}(2));
            p{i+1}(3)=min(p{i}(3),p{i+1}(3));
            p{i+1}(4)=max(p{i}(4),p{i+1}(4)); 
            n=n+1;
            ncount(n)=i+1;
        end
    end
  
    if(n>0)
        d_ncount=ncount(2:n+1)-ncount(1:n);
        index=find(d_ncount~=1);
        m=length(index);
        for i=1:m
            pp{i}=p{ncount(index(i))};
           
            %pp_center{i}=p_center{ncount(i)};
           
           
            pp_ratio(i)=(pp{i}(4)-pp{i}(3))/(pp{i}(2)-pp{i}(1));    
        end
        p=pp;
        p_ratio=pp_ratio;
        clear pp;clear pp_ratio;
    end
end


k=length(p);



m=1;T=0.6*max(p_ratio);     
for i=1:k
    if(p_ratio(i)>=T&p_ratio(i)<20)
        p1{m}=p{i};
        m=m+1;
    end
end
p=p1;
%clear p1;
k=m-1;

toc                            

clear edge_IM3;clear x; clear y;

figure(10);
for i=1:k
%     subplot(1,k,i);
    index=p{i};
    imshow(I(index(1)-2:index(2),index(3):index(4)));
    imwrite(I(index(1)-2:index(2),index(3):index(4)),'plate.jpg');
end


% Finding the angle of output image
gray_image = imread('E:\work\plate.jpg');
 % figure(1), imshow(gray_image);
theta = (0:179)';
 % Determining the lines on the picture using the Radon Transform:
 [R, xp] = radon(edge(gray_image), theta);

 % Determining the largest lines on the picture:
 i = find(R > (max(R(:)) - 25));
 [foo, ind] = sort(-R(i));
 [y, x] = ind2sub(size(R), i);
 t = -theta(x)*pi/180;
 r = xp(y);

 % Forming a matrix representation of the found lines:
 lines = [cos(t) sin(t) -r];
cx = size(gray_image, 2)/2 - 1;
cy = size(gray_image, 1)/2 - 1;
 lines(:,3) = lines(:,3) - lines(:,1)*cx - lines(:,2)*cy;

 % Finding the angle of the most visible line on the picture:
 [r,c] = find(R == max(R(:)));
 thetap = theta(c(1));
angle = 90 - thetap;


% P1=imread('E:\work\plate.jpg');
 % % figure(1), imshow(P1),title('Original Image');
 level=graythresh(gray_image);
 P2=im2bw(gray_image,level);
      
 myI=edge(P2,'sobel');
 % theta=0:179;
 [r1,x1]=radon(myI,theta);
 R1=sort(r1,'descend');
 [x,y]=size(R1);
 R2=R1(1:20,:);
 R=sum(R2);
       
 a=0.4;
E(1)=R(1);
for i=2:length(R)
    E(i)=a*R(i)+(1-a)*E(i-1);
end
a=tan((thetap-(find(E==max(E))))*pi/180);
P3=imrotate(P2,a);
figure(11), imshow(P3);
% P4=~P3;
% figure(3), imshow(P4);
 
