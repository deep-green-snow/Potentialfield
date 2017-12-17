map=[[1 1 1 1 1 1];[1 1 1 1 1 1];[1 0 0 0 0 0];[1 1 0 0 1 1];[1 1 1 1 1 1]];
obs=[0 0];
goal=[0 0];
index = 1;
index2 = 1;
Kg=80;
Ks=2;
K1=80;
K2=80;
K3=80;
K4=80;
K5=80;
K6=80;
K7=80;
K8=80;
K9=80;
K10=80;
K11=80;
K12=80;
K13=80;
K14=80;
K15=80;
K16=80;
K17=80;
K18=80;
K19=80;
K20=80;
K21=80;
K22=80;
K23=80;


for x=1:5
    for y=1:6
        de = map(x,y);
        if de == 1
            obs(index,:)=[x y];
            index = index + 1; 
        else if de == 0
            if x==2||x==4
                goal(index2,:)=[x y];
                index2 = index2 + 1;
            end
            end
        end
    end
end

obs1=obs(1,:);
obs2=obs(2,:);
obs3=obs(3,:);
obs4=obs(4,:);
obs5=obs(5,:);
obs6=obs(6,:);
obs7=obs(7,:);
obs8=obs(8,:);
obs9=obs(9,:);
obs10=obs(10,:);
obs11=obs(11,:);
obs12=obs(12,:);
obs13=obs(13,:);
obs14=obs(14,:);
obs15=obs(15,:);
obs16=obs(16,:);
obs17=obs(17,:);
obs18=obs(18,:);
obs19=obs(19,:);
obs20=obs(20,:);
obs21=obs(21,:);
obs22=obs(22,:);
obs23=obs(23,:);

goal1=goal(1,:);
goal2=goal(2,:);
start=[3,6];

X=1:50;
Y=1:60;
A = zeros(5,6);



for i=1:50
    for j=1:60
       R1=(j/10-obs1(2))^2+(i/10-obs1(1))^2;
       R2=(j/10-obs2(2))^2+(i/10-obs2(1))^2;
       R3=(j/10-obs3(2))^2+(i/10-obs3(1))^2;
       R4=(j/10-obs4(2))^2+(i/10-obs4(1))^2;
       R5=(j/10-obs5(2))^2+(i/10-obs5(1))^2;
       R6=(j/10-obs6(2))^2+(i/10-obs6(1))^2;
       R7=(j/10-obs7(2))^2+(i/10-obs7(1))^2;
       R8=(j/10-obs8(2))^2+(i/10-obs8(1))^2;
       R9=(j/10-obs9(2))^2+(i/10-obs9(1))^2;
       R10=(j/10-obs10(2))^2+(i/10-obs10(1))^2;
       R11=(j/10-obs11(2))^2+(i/10-obs11(1))^2;
       R12=(j/10-obs12(2))^2+(i/10-obs12(1))^2;
       R13=(j/10-obs13(2))^2+(i/10-obs13(1))^2;
       R14=(j/10-obs14(2))^2+(i/10-obs14(1))^2;
       R15=(j/10-obs15(2))^2+(i/10-obs15(1))^2;
       R16=(j/10-obs16(2))^2+(i/10-obs16(1))^2;
       R17=(j/10-obs17(2))^2+(i/10-obs17(1))^2;
       R18=(j/10-obs18(2))^2+(i/10-obs18(1))^2;
       R19=(j/10-obs19(2))^2+(i/10-obs19(1))^2;
       R20=(j/10-obs20(2))^2+(i/10-obs20(1))^2;
       R21=(j/10-obs21(2))^2+(i/10-obs21(1))^2;
       R22=(j/10-obs22(2))^2+(i/10-obs22(1))^2;
       R23=(j/10-obs23(2))^2+(i/10-obs23(1))^2;
       
       Start = (j/10-start(2))^2+(i/10-start(1))^2;
       RG1 = sqrt((j/10-goal1(2))^2+(i/10-goal1(1))^2);
       RG2 = sqrt((j/10-goal2(2))^2+(i/10-goal2(1))^2);
       A(i,j)=Kg*(RG1+RG2)+Ks/Start+K1/(R1)+K2/(R2)+K3/(R3)+K4/(R4)+K5/(R5)+K6/(R6)+K7/(R7)+K8/(R8)+K9/(R9)+K10/(R10)+K11/(R11)+K12/(R12)+K13/(R13)+K14/(R14)+K15/(R15)+K16/(R16)+K17/(R17)+K18/(R18)+K19/(R19)+K20/(R20)+K21/(R21)+K22/(R22)+K23/(R23);
       if A(i,j)>1050
           A(i,j)=1050;
       end
    end
end

a = 30;
b = 57;
GoalX = 38;
GoalY = 35;
distance = sqrt(abs(a-GoalX)^2+abs(b-GoalY)^2);
pathx=[0];
pathy=[0];
k=1;

while(distance>0)
    [a1,b1] = Pathplanning(A,a,b,50,60);
    pathx(k) = a;
    pathy(k) = b;   
    a=a1;
    b=b1;
    
    distance = sqrt(abs(a-GoalX)^2+abs(b-GoalY)^2);
       
    k=k+1;
end

a2 = 30;
b2 = 20;
pathx2=[0];
pathy2=[0];
distance2 = sqrt(abs(a2-GoalX)^2+abs(b2-GoalY)^2);
k=1;

while(distance2>0)
    [a1,b1] = Pathplanning(A,a2,b2,50,60);
    pathx2(k) = a2;
    pathy2(k) = b2;   
    a2=a1;
    b2=b1;
    distance2 = sqrt(abs(a2-GoalX)^2+abs(b2-GoalY)^2);
    k=k+1;
end


figure(1)
surf(Y,X,A)
figure(2)
contour(Y,X,A(X,Y),60)
hold on
[px,py]=gradient(A);
quiver(Y,X,-px,-py,2)
figure(3)
contour(Y,X,A(X,Y),60)
hold on
plot(pathy,pathx,'ko')
figure(4)
contour(Y,X,A(X,Y),60)
hold on
plot(pathy2,pathx2,'bo')

    
    





