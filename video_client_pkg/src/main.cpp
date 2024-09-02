#include <iostream>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

extern "C" {
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
}

// Declaración de variables globales para el nodo y el publisher
rclcpp::Node::SharedPtr node = nullptr;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_ = nullptr;


//Función para publicar el frame decodificado a un topic de ROS 2
void publish_frame(AVCodecContext *pOCodecCtx, AVFrame *pFrame) {

   int width = pOCodecCtx->width;
    int height = pOCodecCtx->height;

  
    SwsContext *sws_ctx = sws_getContext(
        width, height, pOCodecCtx->pix_fmt,
        width, height, AV_PIX_FMT_GRAY8,  // Escala a formato de un canal de 8 bits
        SWS_BILINEAR, nullptr, nullptr, nullptr
    );

    // Crear una imagen OpenCV para almacenar la imagen en escala de grises
    cv::Mat img(height, width, CV_8UC1);  


    uint8_t *dest[4] = { img.data, nullptr, nullptr, nullptr };
    int dest_linesize[4] = { img.step, 0, 0, 0 };
    sws_scale(sws_ctx, pFrame->data, pFrame->linesize, 0, height, dest, dest_linesize);


    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", img).toImageMsg();

 
    publisher_->publish(*msg);

 
    sws_freeContext(sws_ctx);
    av_frame_free(&pFrame);
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
// Función que retorna el codec del streamer en base al parámetro proporcionado
AVCodec return_steamer_codec(const char *streamer_type){
    if(strcmp(streamer_type, "mjpeg") == 0){
        return *avcodec_find_decoder(AV_CODEC_ID_MJPEG);
    } else if(strcmp(streamer_type, "h264") == 0){
        return *avcodec_find_decoder(AV_CODEC_ID_H264);
    }
    return *avcodec_find_decoder(AV_CODEC_ID_NONE);
}

int main(int argc, char **argv) {
    if(argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <streamer_type>" << std::endl;
        return -1;
    }

    if(strcmp(argv[1], "mjpeg") != 0 && strcmp(argv[1], "h264") != 0){
        std::cerr << "Invalid streamer type. Valid types are: mjpeg, h264" << std::endl;
        return -1;
    }

    // Inicializar ROS 2
    rclcpp::init(argc, argv);

    // Crear el nodo y el publisher
    node = rclcpp::Node::make_shared("video_client_node");
    publisher_ = node->create_publisher<sensor_msgs::msg::Image>("/images/frame_image", rclcpp::SensorDataQoS());

    std::cout << "Starting video client node" << std::endl;

    const char *url = "http://localhost:8080/stream?topic=/cameras/left_fisheye_image/image&qos_profile=sensor_data";
    AVFormatContext *pFormatCtx = nullptr;
    AVCodecContext *pCodecCtx = nullptr;
    const AVCodec *pCodec = nullptr;
    AVFrame *pFrame = nullptr;
    AVPacket packet;
    int videoStream, ret;

    // Inicializar FFmpeg
    avformat_network_init();

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

    // Encontrar el stream de video H.264
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

    pFrame = av_frame_alloc();
    int frameCount = 0;

    // Leer frames del stream
    while (rclcpp::ok() && av_read_frame(pFormatCtx, &packet) >= 0) {
        if (packet.stream_index == videoStream) {
            int response = avcodec_send_packet(pCodecCtx, &packet);
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

                // Publicar la imagen decodificada

                std::cout << "Frame decoded, Frame data: frame count: " << frameCount << " Packet size:" <<  packet.size << " Time (seconds): " << packet.pts  * av_q2d(pFormatCtx->streams[videoStream]->time_base) << std::endl;
                publish_frame(pCodecCtx, pFrame);
            }
        }

        av_packet_unref(&packet);
    }

    av_frame_free(&pFrame);
    avcodec_free_context(&pCodecCtx);
    avformat_close_input(&pFormatCtx);

    rclcpp::shutdown();

    return 0;
}