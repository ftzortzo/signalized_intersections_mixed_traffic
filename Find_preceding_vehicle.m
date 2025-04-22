

if CAVs(i).path==1 || CAVs(i).path==4 || CAVs(i).path==7 || CAVs(i).path==10

    min_diff=100^2;

    for j=1:number_of_CAVs
        if i~=j
            if CAVs(i).path == CAVs(j).path

                if CAVs(i).p > CAVs(j).p

                    if CAVs(i).p - CAVs(j).p < min_diff
                        min_diff = CAVs(i).p - CAVs(j).p;
                        CAVs(i).Preceding_CAV=j;
                    end
                end

            end
        end
    end


elseif (CAVs(i).path==2 || CAVs(i).path==5 || CAVs(i).path==8 || CAVs(i).path==11) && CAVs(i).p>=17
    min_diff=100^2;

    for j=1:number_of_CAVs
        if i~=j
            if CAVs(i).path == CAVs(j).path

                if CAVs(i).p > CAVs(j).p
                    if (CAVs(j).Turn ~= CAVs(i).Turn && CAVs(j).p>10) || (CAVs(j).Turn == CAVs(i).Turn) %If I have different turn, and the preceding vehicle has not turned, I am checking. %I have deducted 10 from 28.25 to allow some space.

                        if CAVs(i).p - CAVs(j).p < min_diff
                            min_diff = CAVs(i).p - CAVs(j).p;
                            CAVs(i).Preceding_CAV=j;

                        end
                    end
                end

            end
        end
    end


elseif (CAVs(i).path==2 || CAVs(i).path==5 || CAVs(i).path==8 || CAVs(i).path==11) && CAVs(i).p<17
    min_diff=100^2;


    for j=1:number_of_CAVs
        if i~=j


            if CAVs(i).Turn=='straight' && CAVs(j).Turn == 'right' && ~(CAVs(j).x<20 && CAVs(j).x>-20 && CAVs(j).y>-20 && CAVs(j).y<20) %the last parenthesis quarantees that I am not checking as a preceding vehicle, a vehicle that is right and stopped at the traffic light.
                continue;
            else
                if ((CAVs(i).path == 11 && CAVs(i).Turn == "straight") || (CAVs(i).path == 2 && CAVs(i).Turn == "right")) && ((CAVs(j).path == 2 && CAVs(j).Turn == "right") || (CAVs(j).path == 11 && CAVs(j).Turn == "straight"))


                    if CAVs(i).x < CAVs(j).x

                        if CAVs(j).x - CAVs(i).x < min_diff
                            min_diff = CAVs(j).x - CAVs(i).x;
                            CAVs(i).Preceding_CAV=j;
                        end
                    end

                elseif ( (CAVs(i).path == 5 && CAVs(i).Turn == "right") || (CAVs(i).path == 2 && CAVs(i).Turn == "straight")) && ((CAVs(j).path == 2 && CAVs(j).Turn == "straight") || (CAVs(j).path == 5 && CAVs(j).Turn == "right"))


                    if CAVs(i).y < CAVs(j).y
                        if CAVs(j).y - CAVs(i).y < min_diff
                            min_diff = CAVs(j).y - CAVs(i).y;
                            CAVs(i).Preceding_CAV=j;
                        end
                    end

                elseif ((CAVs(i).path == 5 && CAVs(i).Turn == "straight") || (CAVs(i).path == 8 && CAVs(i).Turn == "right")) && ((CAVs(j).path == 5 && CAVs(j).Turn == "straight") || (CAVs(j).path == 8 && CAVs(j).Turn == "right"))


                    if CAVs(i).x > CAVs(j).x
                        if abs(CAVs(i).x - CAVs(j).x) < min_diff
                            min_diff = abs(CAVs(i).x - CAVs(j).x);
                            CAVs(i).Preceding_CAV=j;
                        end
                    end

                elseif ((CAVs(i).path == 11 && CAVs(i).Turn == "right") || (CAVs(i).path == 8 && CAVs(i).Turn == "straight")) && ((CAVs(j).path == 8 && CAVs(j).Turn == "straight") || (CAVs(j).path == 11 && CAVs(j).Turn == "right"))


                    if CAVs(i).y > CAVs(j).y
                        if abs(CAVs(i).y - CAVs(j).y) < min_diff
                            min_diff = abs(CAVs(i).y - CAVs(j).y);
                            CAVs(i).Preceding_CAV=j;
                        end
                    end

                end
           

            end

        end
    end

end



