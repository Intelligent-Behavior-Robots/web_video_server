#include <iostream>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <vector>

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

    while (true) {
        // Intentar recibir un paquete del codificador
        if (avcodec_receive_packet(pOCodecCtx, &pkt) < 0) {
            // Si no se recibió un paquete, salir del bucle
            break;
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
    }

    avcodec_close(pOCodecCtx);
    avcodec_free_context(&pOCodecCtx);
}

int main(int argc, char **argv) {
    const char *url = "http://localhost:8080/stream?topic=/cameras/frontright_fisheye_image/image";
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

    // Obtener el codec de video H.264
    pCodec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!pCodec) {
        std::cerr << "Codec H.264 no encontrado" << std::endl;
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
        std::cerr << "No se pudo abrir el codec H.264" << std::endl;
        avcodec_free_context(&pCodecCtx);
        avformat_close_input(&pFormatCtx);
        return -1;
    }

    pFrame = av_frame_alloc();
    int frameCount = 0;

    // Leer frames del stream
    while (av_read_frame(pFormatCtx, &packet) >= 0) {
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

                std::cout << "Frame decoded, saving as JPEG, frame count: " << frameCount << std::endl;

                // Guardar el frame como JPEG
                save_frame_as_jpeg(pFrame, frameCount++);
            }
        }

        av_packet_unref(&packet);
    }

    av_frame_free(&pFrame);
    avcodec_free_context(&pCodecCtx);
    avformat_close_input(&pFormatCtx);

    return 0;
}
