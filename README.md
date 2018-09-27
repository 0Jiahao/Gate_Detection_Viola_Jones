# Gate_Detection_Viola_Jones
for tl = 1:size(tl_bbox,1)  
        tl_l = tl_bbox(tl,1);  
        tl_r = tl_bbox(tl,1) + tl_bbox(tl,3);  
        tl_t = tl_bbox(tl,2);  
        tl_b = tl_bbox(tl,2) + tl_bbox(tl,4);  
        for tr = 1:size(tr_bbox,1)  
           tr_l = tr_bbox(tr,1);  
           tr_r = tr_bbox(tr,1) + tr_bbox(tr,3);  
           tr_t = tr_bbox(tr,2);  
           tr_b = tr_bbox(tr,2) + tr_bbox(tr,4);  
           if (tl_r < tr_l) && (tl_b > tr_t) && (tl_t < tr_b)  
               for br = 1:size(br_bbox,1)  
                   br_l = br_bbox(br,1);  
                   br_r = br_bbox(br,1) + br_bbox(br,3);  
                   br_t = br_bbox(br,2);  
                   br_b = br_bbox(br,2) + br_bbox(br,4);  
                   if (tr_b < br_t) && (tr_l < br_r) && (tr_r > tr_l)  
                       for bl = 1:size(bl_bbox,1)  
                           bl_l = bl_bbox(bl,1);  
                           bl_r = bl_bbox(bl,1) + bl_bbox(bl,3);  
                           bl_t = bl_bbox(bl,2);  
                           bl_b = bl_bbox(bl,2) + bl_bbox(bl,4);  
                           if (br_l > bl_r) && (br_t < bl_b) && (br_b > bl_t) && (tl_b < bl_t) && (tl_l< bl_r) && (tl_r > bl_l)    
                               rectangles = [rectangles;[tl,tr,br,bl]];  
                           end   
                       end  
                   end  
               end  
           end  
        end  
    end  
