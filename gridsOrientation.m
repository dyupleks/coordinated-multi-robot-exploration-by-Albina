function [V1,V2,V3,V4,V5,V6,V7,V8,V9] = gridsOrientation(x,y)
    V1 = [x,y+1];
    
    
    V2 = [x+1,y+1];
    V3 = [x+1,y];
    
    V4 = [x+1,y-1];

    
    
    V5 = [x,y-1];
    
    
    V6 = [x-1,y-1];
    
    
    V7 = [x-1,y];

    
    V8 = [x-1,y+1];

    
    V9 = [x,y];
   
    
end