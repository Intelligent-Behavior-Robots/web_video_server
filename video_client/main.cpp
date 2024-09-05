#include <iostream>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <vector>
#include <ctime>

extern "C" {
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>

}


// Esta función guarda un frame como JPEG
void save_frame_as_jpeg(AVFrame *pFrame, int FrameNo) {
    AVCodecContext *pOCodecCtx;
    const AVCodec *pOCodec;
    AVPacket pkt;
    av_init_packet(&pkt);

    // Encuentra el encoder de JPEG
    pOCodec = avcodec_find_encoder(AV_CODEC_ID_MJPEG);
    if (!pOCodec) {
        std::cerr << "Codec not found" << std::endl;
        return;
    }
    pOCodecCtx = avcodec_alloc_context3(pOCodec);
    if (!pOCodecCtx) {
        std::cerr << "Could not allocate video codec context" << std::endl;
        return;
    }

    pOCodecCtx->bit_rate = 150000;
    pOCodecCtx->width = pFrame->width;
    pOCodecCtx->height = pFrame->height;
    pOCodecCtx->pix_fmt = AV_PIX_FMT_YUVJ420P;
    pOCodecCtx->codec_id = AV_CODEC_ID_MJPEG;
    pOCodecCtx->codec_type = AVMEDIA_TYPE_VIDEO;
    pOCodecCtx->time_base = (AVRational){1, 25};

    // Open the codec
    if (avcodec_open2(pOCodecCtx, pOCodec, NULL) < 0) {
        std::cerr << "Could not open codec" << std::endl;
        avcodec_free_context(&pOCodecCtx);
        return;
    }

    // Enviar el frame al codificador
    if (avcodec_send_frame(pOCodecCtx, pFrame) < 0) {
        std::cerr << "Error al enviar el frame al codificador." << std::endl;
    }


        // Intentar recibir un paquete del codificador
        if (avcodec_receive_packet(pOCodecCtx, &pkt) < 0) {
            // Si no se recibió un paquete, salir del bucle
            std::cerr << "Error al recibir el paquete del codificador." << std::endl;
        }

        char filename[32];
        sprintf(filename, "frame%d.jpg", FrameNo);
        std::ofstream outFile(filename, std::ios::out | std::ios::binary);
        if (!outFile) {
            std::cerr << "Error opening output file: " << filename << std::endl;
            av_packet_unref(&pkt);
            return;
        }
        std::cout << "Saving frame as JPEG: " << filename << std::endl;
        outFile.write((char *)pkt.data, pkt.size);
        outFile.close();
        av_packet_unref(&pkt);
    

    avcodec_close(pOCodecCtx);
    avcodec_free_context(&pOCodecCtx);
}

//Funcion que retorna el codec del streamer en base al parametro proporcionado
AVCodec return_steamer_codec(const char *streamer_type){
    if(strcmp(streamer_type, "mjpeg") == 0){
        return *avcodec_find_decoder(AV_CODEC_ID_MJPEG);
    }

    else if(strcmp(streamer_type, "h264") == 0){
        return *avcodec_find_decoder(AV_CODEC_ID_H264);
    }
}


void print_avpacket(const AVPacket *pkt) {
    std::cout << "AVPacket:" << std::endl;
    std::cout << "  buf: " << (void *)pkt->buf << std::endl;
    std::cout << "  pts: " << pkt->pts << std::endl;
    std::cout << "  dts: " << pkt->dts << std::endl;
    std::cout << "  data: " << (void *)pkt->data << std::endl;
    std::cout << "  size: " << pkt->size << std::endl;
    std::cout << "  stream_index: " << pkt->stream_index << std::endl;
    std::cout << "  flags: " << pkt->flags << std::endl;
    std::cout << "  side_data: " << (void *)pkt->side_data << std::endl;
    std::cout << "  side_data_elems: " << pkt->side_data_elems << std::endl;
    std::cout << "  duration: " << pkt->duration << std::endl;
    std::cout << "  pos: " << pkt->pos << std::endl;
}

void printCodecContextInfo(AVCodecContext* codecCtx) {
    std::cout << "Codec Context Information:" << std::endl;
    std::cout << "Codec Name: " << codecCtx->codec->name << std::endl;
    std::cout << "Codec Type: " << av_get_media_type_string(codecCtx->codec_type) << std::endl;
    std::cout << "Codec ID: " << codecCtx->codec_id << std::endl;
    std::cout << "Codec Bit Rate: " << codecCtx->bit_rate << std::endl;
    std::cout << "Codec Width: " << codecCtx->width << std::endl;
    std::cout << "Codec Height: " << codecCtx->height << std::endl;
    
}

void printCodecInfo(const AVCodec *pCodec) {
    if (pCodec) {
        std::cout << "Codec Information:" << std::endl;
        std::cout << "Name: " << pCodec->name << std::endl;
        std::cout << "Long Name: " << pCodec->long_name << std::endl;
        std::cout << "Type: " << av_get_media_type_string(pCodec->type) << std::endl;
        std::cout << "ID: " << pCodec->id << std::endl;
        std::cout << "Capabilities: " << pCodec->capabilities << std::endl;
    } else {
        std::cout << "Codec is nullptr" << std::endl;
    }
}

void printFormatContextInfo(AVFormatContext *pFormatCtx) {
    if (pFormatCtx) {
        std::cout << "Format Context Information:" << std::endl;
        std::cout << "Duration: " << pFormatCtx->duration << std::endl;
        std::cout << "Bit Rate: " << pFormatCtx->bit_rate << std::endl;
        std::cout << "Start Time: " << pFormatCtx->start_time << std::endl;
        std::cout << "Number of Streams: " << pFormatCtx->nb_streams << std::endl;
        std::cout << "Format Name: " << pFormatCtx->iformat->name << std::endl;
        std::cout << "Format Long Name: " << pFormatCtx->iformat->long_name << std::endl;
    } else {
        std::cout << "Format Context is nullptr" << std::endl;
    }
}

int main(int argc, char **argv) {
    
    
    if(argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <streamer_type>" << std::endl;
        return -1;
    }

    else {

        if(strcmp(argv[1], "mjpeg") != 0 && strcmp(argv[1], "h264") != 0){
            std::cerr << "Invalid streamer type. Valid types are: mjpeg, h264" << std::endl;
            return -1;
        }
    }


    const char *url = "http://192.168.2.239:8080/stream?topic=/cameras/left_fisheye_image/image&qos_profile=sensor_data";
    AVFormatContext *pFormatCtx = nullptr;
    AVCodecContext *pCodecCtx = nullptr;
    const AVCodec *pCodec = nullptr;
    AVFrame *pFrame = nullptr;
    AVPacket packet;
    int videoStream, ret;

    // Inicializar FFmpeg
    avformat_network_init();

   

    std::cout << "Iniciando stream desde la URL: " << url << std::endl;

    // Abrir el stream desde la URL
    if (avformat_open_input(&pFormatCtx, url, nullptr, nullptr) != 0) {
        std::cerr << "No se pudo abrir el stream desde la URL" << std::endl;
        return -1;
    }

    // Obtener información del stream
    if (avformat_find_stream_info(pFormatCtx, nullptr) < 0) {
        std::cerr << "No se pudo encontrar la información del stream" << std::endl;
        avformat_close_input(&pFormatCtx);
        return -1;
    }

    // Encontrar el stream de video 
    videoStream = -1;
    for (unsigned i = 0; i < pFormatCtx->nb_streams; i++) {
        if (pFormatCtx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            videoStream = i;
            break;
        }
    }

    if (videoStream == -1) {
        std::cerr << "No se encontró stream de video" << std::endl;
        avformat_close_input(&pFormatCtx);
        return -1;
    }

    // Obtener el codec de video
    auto codec = return_steamer_codec(argv[1]);
    pCodec = &codec;
 
    if (!pCodec) {
        std::cerr << "Codec no encontrado" << std::endl;
        avformat_close_input(&pFormatCtx);
        return -1;
    }

    pCodecCtx = avcodec_alloc_context3(pCodec);
    if (!pCodecCtx) {
        std::cerr << "No se pudo asignar el contexto del codec" << std::endl;
        avformat_close_input(&pFormatCtx);
        return -1;
    }





    if (avcodec_parameters_to_context(pCodecCtx, pFormatCtx->streams[videoStream]->codecpar) < 0) {
        std::cerr << "No se pudo copiar los parámetros del codec" << std::endl;
        avcodec_free_context(&pCodecCtx);
        avformat_close_input(&pFormatCtx);
        return -1;
    }

    if (avcodec_open2(pCodecCtx, pCodec, nullptr) < 0) {
        std::cerr << "No se pudo abrir el codec especificado" << std::endl;
        avcodec_free_context(&pCodecCtx);
        avformat_close_input(&pFormatCtx);
        return -1;
    } 
    
    //Cuando se ejecuta en android se quejaba de no tener el time_base
    pCodecCtx->time_base = pFormatCtx->streams[videoStream]->time_base;
    pFrame = av_frame_alloc();
    int frameCount = 0;

    printCodecInfo(pCodec);
    printFormatContextInfo(pFormatCtx);


  

    // Leer frames del stream
    while (av_read_frame(pFormatCtx, &packet) >= 0) {
        if (packet.stream_index == videoStream) {
            int response = avcodec_send_packet(pCodecCtx, &packet);
            print_avpacket(&packet);
            printCodecContextInfo(pCodecCtx);

            std::cout << "Response: " << response << std::endl;
            if (response < 0) {
                std::cerr << "Error al enviar el paquete al decodificador" << std::endl;
                break;
            }

            while (response >= 0) {
                response = avcodec_receive_frame(pCodecCtx, pFrame);
                if (response == AVERROR(EAGAIN) || response == AVERROR_EOF) {
                    break;
                } else if (response < 0) {
                    std::cerr << "Error al recibir un frame del decodificador" << std::endl;
                    break;
                }

              //std::cout << "Frame decoded, saving as JPEG, frame count: " << frameCount << " Packet size:" <<  packet.size << " Time (seconds): " << packet.pts  * av_q2d(pFormatCtx->streams[videoStream]->time_base) << std::endl;

                // Guardar el frame como JPEG
               // save_frame_as_jpeg(pFrame, frameCount++);
            }
        }

        av_packet_unref(&packet);
    }

    av_frame_free(&pFrame);
    avcodec_free_context(&pCodecCtx);
    avformat_close_input(&pFormatCtx);

    return 0;
}
