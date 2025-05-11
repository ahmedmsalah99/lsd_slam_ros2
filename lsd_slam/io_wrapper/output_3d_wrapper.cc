#include "output_3d_wrapper.h"


namespace lsd_slam
{


void Output3DWrapper::setSocket(int output_client_fd)
{
    sock_fd_ = output_client_fd;
}

void Output3DWrapper::sendBufferData(std::vector<uchar> buffer){
    try {
        // Send image size first (as 4-byte integer)
        uint32_t size = buffer.size();
        uint32_t size_net = htonl(size);
        
        if (send(sock_fd_, &size_net, sizeof(size_net), 0) < 0) {
            std::cerr << "Failed to send keyframe data size\n";
            close(sock_fd_);
            return;
        }
        // Send image data
        if (send(sock_fd_, buffer.data(), buffer.size(), 0) < 0) {
            std::cerr << "Failed to send keyframe data\n";
            close(sock_fd_);
            return;
        }
        std::cout << "Sent keyframe data of size : "<< size <<" bytes\n";
    } catch (...) {
        std::cerr << "keyframe data sending exception: ";
    }
}



void Output3DWrapper::publishKeyframe(Frame* f) {
        boost::shared_lock<boost::shared_mutex> lock = f->getActiveLock();
        CloudPoints3D fMsg{};
        std::cout << "publishing key frame\n"; 
        // extract frame data
        int publishLvl = 0;
        fMsg.id = f->id();
        fMsg.time = f->timestamp();

        int width = f->width(publishLvl);
        int height = f->height(publishLvl);

        float fx = f->fx(publishLvl);
        float fy = f->fy(publishLvl);
        float cx = f->cx(publishLvl);
        float cy = f->cy(publishLvl);

        float fxi = 1/fx;
        float fyi = 1/fy;
        float cxi = -cx / fx;
        float cyi = -cy / fy;
        const float* idepth = f->idepth(publishLvl);
	    const float* idepthVar = f->idepthVar(publishLvl);

        memcpy(fMsg.camToWorld,f->getScaledCamToWorld().cast<float>().data(),sizeof(float)*7);

        float my_scale = f->getScaledCamToWorld().cast<float>().scale();
        
        int my_scaledTH = 1;
        int my_absTH = 1;
        int my_sparsifyFactor = 1;
        int minNearSupport = 5;
        int vertexBufferNumPoints = 0;
        CloudPoint3D* tmpBuffer = new CloudPoint3D[width*height];
        std::cout << "initialization done" << std::endl;
        std::cout << "params are w " << width << " h " << height << " fx " << fx << " fxi " << fxi << " cxi " << cxi << std::endl;
        for(int y=1;y<height-1;y++){
            if(vertexBufferNumPoints > MAX_CLOUD_POINTS-1)
            {
                break;
            }
		    for(int x=1;x<width-1;x++)
		    {
                if(idepth[x+y*width]<0)
                    continue;

                if(my_sparsifyFactor > 1 && rand()%my_sparsifyFactor != 0) continue;

                float depth = 1 / idepth[x+y*width];
                
                float depth4 = depth*depth; depth4*= depth4;

                if(idepthVar[x+y*width] * depth4 > my_scaledTH)
                    continue;

                if(idepthVar[x+y*width] * depth4 * my_scale*my_scale > my_absTH)
                    continue;

                if(minNearSupport > 1)
                {
                    int nearSupport = 0;
                    for(int dx=-1;dx<2;dx++)
                        for(int dy=-1;dy<2;dy++)
                        {
                            int idx = x+dx+(y+dy)*width;
                            if(idepth[idx] > 0)
                            {
                                float diff = idepth[idx] - 1.0f / depth;
                                if(diff*diff < 2*idepthVar[x+y*width])
                                    nearSupport++;
                            }
                        }

                    if(nearSupport < minNearSupport)
                        continue;
                }
                if(vertexBufferNumPoints == 0)
                    std::cout << "depth is " << depth << std::endl;
                tmpBuffer[vertexBufferNumPoints].x = (x*fxi + cxi) * depth;
                tmpBuffer[vertexBufferNumPoints].y = (y*fyi + cyi) * depth;
                tmpBuffer[vertexBufferNumPoints].z =  depth;
                vertexBufferNumPoints ++;
                if(vertexBufferNumPoints > MAX_CLOUD_POINTS-1)
                {
                    break;
                }

            }
        }
        fMsg.scale = my_scale;
        fMsg.cloud_points_num = vertexBufferNumPoints;
        std::cout << "there are " << vertexBufferNumPoints << "valid points" << std::endl;
        memcpy(fMsg.cloud_points,tmpBuffer,sizeof(CloudPoint3D)*vertexBufferNumPoints);
        std::cout << "msg cloud points copied" << std::endl;

        std::vector<uchar> buffer(sizeof(CloudPoints3D));
        std::cout << "buffer size " << buffer.size() << " msg size " << sizeof(fMsg) << " Cloud Points size " << sizeof(CloudPoints3D) << std::endl;
        std::memcpy(buffer.data(), &fMsg, sizeof(CloudPoints3D));
        std::cout << "msg is constructed and ready to be published" << std::endl;
        // prints for debugging
        for (auto point : fMsg.camToWorld)
            std::cout << "point " << point << std::endl;
        std::cout << "scale " << my_scale <<std::endl;
        std::cout << "width " << width << std::endl;
        std::cout << "height " << height << std::endl;
        std::cout << "first cloud point " << fMsg.cloud_points[0].x << std::endl;




        sendBufferData(buffer);
    };
    Output3DWrapper::~Output3DWrapper(){

    }
}