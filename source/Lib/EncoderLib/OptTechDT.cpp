#include "OptTechDT.h"

int OptTechDT::width, OptTechDT::height, OptTechDT::numOfFrames, OptTechDT::depthMapAllocSize, OptTechDT::quantPar;
int OptTechDT::depthMaps[DEPTH_MAP_NUM_OF_FRAMES][DEPTH_MAP_MAX_FRAME_SIZE];
int OptTechDT::encoderConfig;

bool OptTechDT::skipCheckRD;

void OptTechDT::init(int w, int h, int nf, bool isLowDelay, int qp) {
    width = w;
    height = h;
    numOfFrames = nf;
    // RA/LD is derived automatically from the GOP/RPL structure (EncAppCfg::m_isLowDelay)
    // instead of being read from the .cfg files.
    // [legacy] encoderConfig = (encCfg == "RA") ? ENCODER_RA_CONFIG : ENCODER_LD_CONFIG;
    encoderConfig = isLowDelay ? ENCODER_LD_CONFIG : ENCODER_RA_CONFIG;
    quantPar = qp;

    // std::cout << "[DBG] Encoder Configuration: " << encoderConfig << (isLowDelay ? " LD" : " RA") << std::endl;

    depthMapAllocSize = (width / DEPTH_MAP_RESOLUTION) * (height / DEPTH_MAP_RESOLUTION);

    for (int f = 0; f < numOfFrames; f++) {
        for (int i = 0; i < DEPTH_MAP_MAX_FRAME_SIZE; i++) {
            depthMaps[f][i] = 0;
        }               
    }      
    skipCheckRD = false;
}

double OptTechDT::calculateDiffVariance(int xBlk, int yBlk, int wBlk, int hBlk, PelUnitBuf origBuff, PelUnitBuf recoBuff) { 
    int startx = xBlk;
    int endx = xBlk + wBlk;
    int starty = yBlk;
    int endy = yBlk + hBlk;

    int sum = 0;

    for (int i = startx; i < endx; i++) {
      for (int j = starty; j < endy; j++) {
        sum += abs((origBuff.Y().at(i,j) >> 2) - (recoBuff.Y().at(i,j) >> 2));
      }
    }

    double mean = (double) sum / (double) (wBlk * hBlk);

    double sqDiff = 0;

    for (int i = startx; i < endx; i++) {
      for (int j = starty; j < endy; j++) {
        sqDiff += abs(abs((origBuff.Y().at(i,j) >> 2) - (recoBuff.Y().at(i,j) >> 2)) - mean) *
                  abs(abs((origBuff.Y().at(i,j) >> 2) - (recoBuff.Y().at(i,j) >> 2)) - mean);
      }
    }

    double variance = (double) sqDiff / (double) (wBlk * hBlk);

    return variance;
}

double OptTechDT::calculateBlockVariance(int xBlk, int yBlk, int wBlk, int hBlk, PelUnitBuf origBuff) {
    int startx = xBlk;
    int endx = xBlk + wBlk;
    int starty = yBlk;
    int endy = yBlk + hBlk;

    int sum = 0;

    for (int i = startx; i < endx; i++) {
      for (int j = starty; j < endy; j++) {
        sum += (origBuff.Y().at(i,j) >> 2);
      }
    }

    double mean = (double) sum / (double) (wBlk * hBlk);

    double sqDiff = 0;

    for (int i = startx; i < endx; i++) {
      for (int j = starty; j < endy; j++) {
        sqDiff += abs((origBuff.Y().at(i,j) >> 2) - mean) *
                  abs((origBuff.Y().at(i,j) >> 2) - mean);
      }
    }

    double variance = (double) sqDiff / (double) (wBlk * hBlk);

    return variance;
}

void OptTechDT::updateDepthMap(int framePoc, int xBlk, int yBlk, int wBlk, int hBlk, int depth) {
    int xBegin = xBlk / DEPTH_MAP_RESOLUTION;
    int yBegin = yBlk / DEPTH_MAP_RESOLUTION;
    int xEnd = (xBlk + wBlk) / DEPTH_MAP_RESOLUTION;
    int yEnd = (yBlk + hBlk) / DEPTH_MAP_RESOLUTION;

    for (int x = xBegin; x < xEnd; x++) {
        for (int y = yBegin; y < yEnd; y++) {
            int pos = x + (y * (width / DEPTH_MAP_RESOLUTION));
            depthMaps[framePoc][pos] = depth;
        }
    }
}

int OptTechDT::isPreviousSplit(int currFramePoc, int xCU, int yCU, int currDepth) {
    int refFramePoc = REFERENCE_FRAME_ORDER[encoderConfig][currFramePoc];

    int pos = xCU + (yCU * (width / DEPTH_MAP_RESOLUTION));
    int refDepth = depthMaps[refFramePoc][pos];

    return (refDepth > currDepth) ? 1 : 0;
}

void OptTechDT::debugVarianceCalculation(int xBlk, int yBlk, int wBlk, int hBlk, PelUnitBuf origBuff, PelUnitBuf recoBuff) {
    int startx = xBlk;
    int endx = xBlk + wBlk;
    int starty = yBlk;
    int endy = yBlk + hBlk;

    std::cout << "Orig:" << std::endl;
    for (int i = startx; i < endx; i++) {
      for (int j = starty; j < endy; j++) {
        std::cout << origBuff.Y().at(i,j) << ";";
      }
      std::cout << std::endl;
    }
    std::cout << "Reco:" << std::endl;
    for (int i = startx; i < endx; i++) {
      for (int j = starty; j < endy; j++) {
        std::cout << recoBuff.Y().at(i,j) << ";";
      }
      std::cout << std::endl;
    }
    std::cout << "BlockVar: " << OptTechDT::calculateBlockVariance(xBlk, yBlk, wBlk, hBlk, origBuff) << std::endl;
    std::cout << "DiffVar: " << OptTechDT::calculateDiffVariance(xBlk, yBlk, wBlk, hBlk, origBuff, recoBuff) << std::endl;
}

void OptTechDT::reportDepthMap(int framePoc) {
    if(depthMaps[framePoc] == NULL) {
        std::cout << "No depth map. Skipping...\n";
        return;
    }
    std::cout << "[DBG] DEPTH MAP REPORT\n";
    std::cout << "Frame " << framePoc << std::endl;
    for (int y = 0; y < (height / DEPTH_MAP_RESOLUTION); y++) {
        for (int x = 0; x < (width / DEPTH_MAP_RESOLUTION); x++) {
            int pos = x + (y * (width / DEPTH_MAP_RESOLUTION));
            std::cout << depthMaps[framePoc][pos] << " ";
        }
        std::cout << std::endl;
    }        
}

PelUnitBuf OptTechDT::getRefPicBuf(int currFramePoc, Slice* slice) {
    int refFramePoc = REFERENCE_FRAME_ORDER[encoderConfig][currFramePoc];

    for (int i = 0; i < MAX_NUM_REF; i++) {
        // dbgRefPics[currFramePoc].insert(slice->getRefPOC(REF_PIC_LIST_0, i));
        if(slice->getRefPOC(REF_PIC_LIST_0, i) == refFramePoc) {
            return slice->getRefPic(REF_PIC_LIST_0, i)->getRecoBuf(PIC_RECONSTRUCTION);
        }
    }

    for (int i = 0; i < MAX_NUM_REF; i++) {
        // dbgRefPics[currFramePoc].insert(slice->getRefPOC(REF_PIC_LIST_1, i));
        if(slice->getRefPOC(REF_PIC_LIST_1, i) == refFramePoc) {
            return slice->getRefPic(REF_PIC_LIST_1, i)->getRecoBuf(PIC_RECONSTRUCTION);
        }
    }

    std::cout << "[ERR] No reference picture found!\n";
    return slice->getRefPic(REF_PIC_LIST_0, 0)->getRecoBuf(PIC_RECONSTRUCTION);
}

void OptTechDT::performModelDT(int currQtDepth, int ft_qp, double ft_blockVar, double ft_diffVar, int ft_previousSplit, int ft_height, int ft_config) {
#if ENABLE_TIME_PROFILE
  TimeProfiler::start(DT_MODEL);
#endif

    switch(currQtDepth) {
        case 0:
            if (ft_previousSplit <= 0.5000) {
                if (ft_config <= 0.5000) {
                    if (ft_diffVar <= 51.9689) {
                        if (ft_qp <= 24.5000) {
                            if (ft_diffVar <= 4.0499) {
                                skipCheckRD = true;
                            } else {  // if ft_diffVar > 4.0499
                                skipCheckRD = true;
                            }
                        } else {  // if ft_qp > 24.5000
                            if (ft_diffVar <= 10.9377) {
                                skipCheckRD = true;
                            } else {  // if ft_diffVar > 10.9377
                                if (ft_qp <= 29.5000) {
                                    skipCheckRD = true;
                                } else {  // if ft_qp > 29.5000
                                    skipCheckRD = true;
                                }
                            }
                        }
                    } else {  // if ft_diffVar > 51.9689
                        if (ft_diffVar <= 322.4163) {
                            if (ft_qp <= 24.5000) {
                                if (ft_height <= 1620.0000) {
                                    skipCheckRD = true;
                                } else {  // if ft_height > 1620.0000
                                    skipCheckRD = true;
                                }
                            } else {  // if ft_qp > 24.5000
                                if (ft_qp <= 29.5000) {
                                    if (ft_diffVar <= 244.7957) {
                                        skipCheckRD = true;
                                    } else {  // if ft_diffVar > 244.7957
                                        skipCheckRD = true;
                                    }
                                } else {  // if ft_qp > 29.5000
                                    if (ft_height <= 780.0000) {
                                        skipCheckRD = false;
                                    } else {  // if ft_height > 780.0000
                                        if (ft_diffVar <= 156.9142) {
                                            skipCheckRD = true;
                                        } else {  // if ft_diffVar > 156.9142
                                            if (ft_blockVar <= 461.3905) {
                                                if (ft_height <= 1620.0000) {
                                                    skipCheckRD = true;
                                                } else {  // if ft_height > 1620.0000
                                                    skipCheckRD = true;
                                                }
                                            } else {  // if ft_blockVar > 461.3905
                                                skipCheckRD = true;
                                            }
                                        }
                                    }
                                }
                            }
                        } else {  // if ft_diffVar > 322.4163
                            if (ft_qp <= 24.5000) {
                                if (ft_blockVar <= 2900.4625) {
                                    skipCheckRD = false;
                                } else {  // if ft_blockVar > 2900.4625
                                    if (ft_diffVar <= 1350.1547) {
                                        skipCheckRD = true;
                                    } else {  // if ft_diffVar > 1350.1547
                                        skipCheckRD = false;
                                    }
                                }
                            } else {  // if ft_qp > 24.5000
                                if (ft_diffVar <= 668.8273) {
                                    if (ft_blockVar <= 1130.2131) {
                                        if (ft_qp <= 34.5000) {
                                            if (ft_qp <= 29.5000) {
                                                if (ft_height <= 1620.0000) {
                                                    skipCheckRD = false;
                                                } else {  // if ft_height > 1620.0000
                                                    skipCheckRD = true;
                                                }
                                            } else {  // if ft_qp > 29.5000
                                                skipCheckRD = true;
                                            }
                                        } else {  // if ft_qp > 34.5000
                                            skipCheckRD = true;
                                        }
                                    } else {  // if ft_blockVar > 1130.2131
                                        skipCheckRD = true;
                                    }
                                } else {  // if ft_diffVar > 668.8273
                                    if (ft_blockVar <= 2077.4677) {
                                        if (ft_blockVar <= 85.3486) {
                                            skipCheckRD = true;
                                        } else {  // if ft_blockVar > 85.3486
                                            if (ft_qp <= 34.5000) {
                                                skipCheckRD = true;
                                            } else {  // if ft_qp > 34.5000
                                                skipCheckRD = true;
                                            }
                                        }
                                    } else {  // if ft_blockVar > 2077.4677
                                        if (ft_diffVar <= 1387.0135) {
                                            skipCheckRD = true;
                                        } else {  // if ft_diffVar > 1387.0135
                                            skipCheckRD = true;
                                        }
                                    }
                                }
                            }
                        }
                    }
                } else {  // if ft_config > 0.5000
                    if (ft_diffVar <= 12.1774) {
                        if (ft_qp <= 24.5000) {
                            if (ft_diffVar <= 2.9830) {
                                if (ft_diffVar <= 0.6289) {
                                    skipCheckRD = true;
                                } else {  // if ft_diffVar > 0.6289
                                    if (ft_height <= 1620.0000) {
                                        skipCheckRD = true;
                                    } else {  // if ft_height > 1620.0000
                                        if (ft_blockVar <= 10.9151) {
                                            skipCheckRD = true;
                                        } else {  // if ft_blockVar > 10.9151
                                            if (ft_blockVar <= 134.4469) {
                                                skipCheckRD = true;
                                            } else {  // if ft_blockVar > 134.4469
                                                skipCheckRD = false;
                                            }
                                        }
                                    }
                                }
                            } else {  // if ft_diffVar > 2.9830
                                if (ft_diffVar <= 3.4987) {
                                    if (ft_blockVar <= 633.8994) {
                                        skipCheckRD = false;
                                    } else {  // if ft_blockVar > 633.8994
                                        skipCheckRD = true;
                                    }
                                } else {  // if ft_diffVar > 3.4987
                                    if (ft_blockVar <= 223.6184) {
                                        if (ft_blockVar <= 46.7237) {
                                            skipCheckRD = false;
                                        } else {  // if ft_blockVar > 46.7237
                                            skipCheckRD = true;
                                        }
                                    } else {  // if ft_blockVar > 223.6184
                                        if (ft_diffVar <= 6.4826) {
                                            skipCheckRD = false;
                                        } else {  // if ft_diffVar > 6.4826
                                            skipCheckRD = false;
                                        }
                                    }
                                }
                            }
                        } else {  // if ft_qp > 24.5000
                            if (ft_diffVar <= 6.8096) {
                                if (ft_qp <= 29.5000) {
                                    if (ft_blockVar <= 20.3433) {
                                        skipCheckRD = true;
                                    } else {  // if ft_blockVar > 20.3433
                                        if (ft_diffVar <= 4.0835) {
                                            skipCheckRD = true;
                                        } else {  // if ft_diffVar > 4.0835
                                            if (ft_blockVar <= 5896.9275) {
                                                skipCheckRD = true;
                                            } else {  // if ft_blockVar > 5896.9275
                                                skipCheckRD = false;
                                            }
                                        }
                                    }
                                } else {  // if ft_qp > 29.5000
                                    skipCheckRD = true;
                                }
                            } else {  // if ft_diffVar > 6.8096
                                if (ft_qp <= 34.5000) {
                                    if (ft_blockVar <= 59.1457) {
                                        if (ft_blockVar <= 32.8772) {
                                            skipCheckRD = true;
                                        } else {  // if ft_blockVar > 32.8772
                                            skipCheckRD = true;
                                        }
                                    } else {  // if ft_blockVar > 59.1457
                                        if (ft_qp <= 29.5000) {
                                            if (ft_blockVar <= 992.1308) {
                                                skipCheckRD = true;
                                            } else {  // if ft_blockVar > 992.1308
                                                if (ft_diffVar <= 7.0172) {
                                                    skipCheckRD = true;
                                                } else {  // if ft_diffVar > 7.0172
                                                    skipCheckRD = false;
                                                }
                                            }
                                        } else {  // if ft_qp > 29.5000
                                            skipCheckRD = true;
                                        }
                                    }
                                } else {  // if ft_qp > 34.5000
                                    skipCheckRD = true;
                                }
                            }
                        }
                    } else {  // if ft_diffVar > 12.1774
                        if (ft_qp <= 24.5000) {
                            if (ft_blockVar <= 133.9388) {
                                if (ft_height <= 1620.0000) {
                                    skipCheckRD = false;
                                } else {  // if ft_height > 1620.0000
                                    if (ft_blockVar <= 70.5804) {
                                        skipCheckRD = true;
                                    } else {  // if ft_blockVar > 70.5804
                                        skipCheckRD = false;
                                    }
                                }
                            } else {  // if ft_blockVar > 133.9388
                                if (ft_blockVar <= 381.8583) {
                                    skipCheckRD = false;
                                } else {  // if ft_blockVar > 381.8583
                                    skipCheckRD = false;
                                }
                            }
                        } else {  // if ft_qp > 24.5000
                            if (ft_blockVar <= 161.1719) {
                                if (ft_qp <= 34.5000) {
                                    if (ft_blockVar <= 48.6125) {
                                        skipCheckRD = true;
                                    } else {  // if ft_blockVar > 48.6125
                                        if (ft_qp <= 29.5000) {
                                            skipCheckRD = true;
                                        } else {  // if ft_qp > 29.5000
                                            skipCheckRD = true;
                                        }
                                    }
                                } else {  // if ft_qp > 34.5000
                                    if (ft_blockVar <= 52.2521) {
                                        skipCheckRD = true;
                                    } else {  // if ft_blockVar > 52.2521
                                        skipCheckRD = true;
                                    }
                                }
                            } else {  // if ft_blockVar > 161.1719
                                if (ft_qp <= 34.5000) {
                                    if (ft_qp <= 29.5000) {
                                        if (ft_height <= 1620.0000) {
                                            if (ft_blockVar <= 266.6111) {
                                                if (ft_diffVar <= 18.5167) {
                                                    skipCheckRD = true;
                                                } else {  // if ft_diffVar > 18.5167
                                                    skipCheckRD = true;
                                                }
                                            } else {  // if ft_blockVar > 266.6111
                                                skipCheckRD = false;
                                            }
                                        } else {  // if ft_height > 1620.0000
                                            if (ft_blockVar <= 2734.8768) {
                                                if (ft_diffVar <= 90.0971) {
                                                    if (ft_blockVar <= 346.9757) {
                                                        skipCheckRD = true;
                                                    } else {  // if ft_blockVar > 346.9757
                                                        skipCheckRD = true;
                                                    }
                                                } else {  // if ft_diffVar > 90.0971
                                                    if (ft_blockVar <= 1197.8006) {
                                                        if (ft_blockVar <= 265.5060) {
                                                            skipCheckRD = false;
                                                        } else {  // if ft_blockVar > 265.5060
                                                            if (ft_diffVar <= 295.7616) {
                                                                skipCheckRD = true;
                                                            } else {  // if ft_diffVar > 295.7616
                                                                skipCheckRD = false;
                                                            }
                                                        }
                                                    } else {  // if ft_blockVar > 1197.8006
                                                        if (ft_diffVar <= 866.6293) {
                                                            skipCheckRD = true;
                                                        } else {  // if ft_diffVar > 866.6293
                                                            skipCheckRD = false;
                                                        }
                                                    }
                                                }
                                            } else {  // if ft_blockVar > 2734.8768
                                                skipCheckRD = true;
                                            }
                                        }
                                    } else {  // if ft_qp > 29.5000
                                        if (ft_diffVar <= 350.7854) {
                                            if (ft_blockVar <= 240.1257) {
                                                if (ft_diffVar <= 69.3401) {
                                                    skipCheckRD = true;
                                                } else {  // if ft_diffVar > 69.3401
                                                    skipCheckRD = true;
                                                }
                                            } else {  // if ft_blockVar > 240.1257
                                                if (ft_height <= 780.0000) {
                                                    skipCheckRD = false;
                                                } else {  // if ft_height > 780.0000
                                                    skipCheckRD = true;
                                                }
                                            }
                                        } else {  // if ft_diffVar > 350.7854
                                            if (ft_blockVar <= 2032.1685) {
                                                skipCheckRD = false;
                                            } else {  // if ft_blockVar > 2032.1685
                                                skipCheckRD = true;
                                            }
                                        }
                                    }
                                } else {  // if ft_qp > 34.5000
                                    if (ft_blockVar <= 401.4879) {
                                        if (ft_diffVar <= 316.7883) {
                                            if (ft_diffVar <= 16.9565) {
                                                skipCheckRD = true;
                                            } else {  // if ft_diffVar > 16.9565
                                                if (ft_height <= 780.0000) {
                                                    skipCheckRD = false;
                                                } else {  // if ft_height > 780.0000
                                                    skipCheckRD = true;
                                                }
                                            }
                                        } else {  // if ft_diffVar > 316.7883
                                            skipCheckRD = true;
                                        }
                                    } else {  // if ft_blockVar > 401.4879
                                        if (ft_diffVar <= 22.7315) {
                                            if (ft_blockVar <= 1738.5591) {
                                                skipCheckRD = true;
                                            } else {  // if ft_blockVar > 1738.5591
                                                skipCheckRD = true;
                                            }
                                        } else {  // if ft_diffVar > 22.7315
                                            if (ft_diffVar <= 86.8778) {
                                                if (ft_blockVar <= 1497.6479) {
                                                    skipCheckRD = true;
                                                } else {  // if ft_blockVar > 1497.6479
                                                    skipCheckRD = false;
                                                }
                                            } else {  // if ft_diffVar > 86.8778
                                                if (ft_blockVar <= 2364.0786) {
                                                    if (ft_diffVar <= 278.9555) {
                                                        if (ft_height <= 780.0000) {
                                                            skipCheckRD = false;
                                                        } else {  // if ft_height > 780.0000
                                                            skipCheckRD = true;
                                                        }
                                                    } else {  // if ft_diffVar > 278.9555
                                                        if (ft_blockVar <= 2034.3407) {
                                                            skipCheckRD = true;
                                                        } else {  // if ft_blockVar > 2034.3407
                                                            skipCheckRD = true;
                                                        }
                                                    }
                                                } else {  // if ft_blockVar > 2364.0786
                                                    skipCheckRD = true;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            } else {  // if ft_previousSplit > 0.5000
    if (ft_blockVar <= 252.3939) {
        if (ft_qp <= 24.5000) {
            if (ft_blockVar <= 51.8617) {
                if (ft_config <= 0.5000) {
                    if (ft_diffVar <= 4.2998) {
                        if (ft_diffVar <= 3.3437) {
                            skipCheckRD = true;
                        } else {  // if ft_diffVar > 3.3437
                            skipCheckRD = true;
                        }
                    } else {  // if ft_diffVar > 4.2998
                        if (ft_diffVar <= 16.2628) {
                            if (ft_height <= 1620.0000) {
                                skipCheckRD = true;
                            } else {  // if ft_height > 1620.0000
                                if (ft_blockVar <= 7.8883) {
                                    skipCheckRD = true;
                                } else {  // if ft_blockVar > 7.8883
                                    skipCheckRD = false;
                                }
                            }
                        } else {  // if ft_diffVar > 16.2628
                            skipCheckRD = false;
                        }
                    }
                } else {  // if ft_config > 0.5000
                    if (ft_diffVar <= 2.5479) {
                        skipCheckRD = true;
                    } else {  // if ft_diffVar > 2.5479
                        if (ft_blockVar <= 12.7765) {
                            if (ft_diffVar <= 5.9046) {
                                if (ft_diffVar <= 4.6067) {
                                    if (ft_blockVar <= 7.0673) {
                                        skipCheckRD = true;
                                    } else {  // if ft_blockVar > 7.0673
                                        skipCheckRD = false;
                                    }
                                } else {  // if ft_diffVar > 4.6067
                                    skipCheckRD = true;
                                }
                            } else {  // if ft_diffVar > 5.9046
                                skipCheckRD = false;
                            }
                        } else {  // if ft_blockVar > 12.7765
                            if (ft_diffVar <= 3.5651) {
                                skipCheckRD = false;
                            } else {  // if ft_diffVar > 3.5651
                                skipCheckRD = false;
                            }
                        }
                    }
                }
            } else {  // if ft_blockVar > 51.8617
                if (ft_diffVar <= 5.2750) {
                    if (ft_config <= 0.5000) {
                        if (ft_diffVar <= 2.9663) {
                            skipCheckRD = true;
                        } else {  // if ft_diffVar > 2.9663
                            skipCheckRD = true;
                        }
                    } else {  // if ft_config > 0.5000
                        if (ft_diffVar <= 3.2070) {
                            if (ft_blockVar <= 243.7732) {
                                skipCheckRD = true;
                            } else {  // if ft_blockVar > 243.7732
                                skipCheckRD = false;
                            }
                        } else {  // if ft_diffVar > 3.2070
                            skipCheckRD = false;
                        }
                    }
                } else {  // if ft_diffVar > 5.2750
                    if (ft_config <= 0.5000) {
                        if (ft_diffVar <= 79.7364) {
                            if (ft_diffVar <= 7.2742) {
                                skipCheckRD = false;
                            } else {  // if ft_diffVar > 7.2742
                                if (ft_blockVar <= 250.5950) {
                                    skipCheckRD = false;
                                } else {  // if ft_blockVar > 250.5950
                                    skipCheckRD = true;
                                }
                            }
                        } else {  // if ft_diffVar > 79.7364
                            skipCheckRD = false;
                        }
                    } else {  // if ft_config > 0.5000
                        if (ft_diffVar <= 72.5180) {
                            skipCheckRD = false;
                        } else {  // if ft_diffVar > 72.5180
                            skipCheckRD = false;
                        }
                    }
                }
            }
        } else {  // if ft_qp > 24.5000
            if (ft_diffVar <= 13.7807) {
                if (ft_diffVar <= 5.6621) {
                    if (ft_qp <= 29.5000) {
                        if (ft_blockVar <= 32.2107) {
                            skipCheckRD = true;
                        } else {  // if ft_blockVar > 32.2107
                            if (ft_blockVar <= 159.0376) {
                                if (ft_diffVar <= 3.6841) {
                                    skipCheckRD = true;
                                } else {  // if ft_diffVar > 3.6841
                                    skipCheckRD = true;
                                }
                            } else {  // if ft_blockVar > 159.0376
                                skipCheckRD = true;
                            }
                        }
                    } else {  // if ft_qp > 29.5000
                        skipCheckRD = true;
                    }
                } else {  // if ft_diffVar > 5.6621
                    if (ft_blockVar <= 64.2946) {
                        if (ft_qp <= 29.5000) {
                            skipCheckRD = true;
                        } else {  // if ft_qp > 29.5000
                            skipCheckRD = true;
                        }
                    } else {  // if ft_blockVar > 64.2946
                        if (ft_qp <= 29.5000) {
                            if (ft_config <= 0.5000) {
                                skipCheckRD = true;
                            } else {  // if ft_config > 0.5000
                                skipCheckRD = false;
                            }
                        } else {  // if ft_qp > 29.5000
                            if (ft_qp <= 34.5000) {
                                if (ft_diffVar <= 7.4350) {
                                    skipCheckRD = true;
                                } else {  // if ft_diffVar > 7.4350
                                    skipCheckRD = true;
                                }
                            } else {  // if ft_qp > 34.5000
                                skipCheckRD = true;
                            }
                        }
                    }
                }
            } else {  // if ft_diffVar > 13.7807
                if (ft_qp <= 29.5000) {
                    if (ft_blockVar <= 74.8446) {
                        if (ft_blockVar <= 29.4531) {
                            skipCheckRD = true;
                        } else {  // if ft_blockVar > 29.4531
                            if (ft_height <= 1620.0000) {
                                skipCheckRD = true;
                            } else {  // if ft_height > 1620.0000
                                skipCheckRD = false;
                            }
                        }
                    } else {  // if ft_blockVar > 74.8446
                        if (ft_diffVar <= 103.8003) {
                            if (ft_config <= 0.5000) {
                                if (ft_diffVar <= 24.8887) {
                                    skipCheckRD = true;
                                } else {  // if ft_diffVar > 24.8887
                                    skipCheckRD = false;
                                }
                            } else {  // if ft_config > 0.5000
                                skipCheckRD = false;
                            }
                        } else {  // if ft_diffVar > 103.8003
                            skipCheckRD = false;
                        }
                    }
                } else {  // if ft_qp > 29.5000
                    if (ft_blockVar <= 65.2714) {
                        if (ft_blockVar <= 28.3802) {
                            skipCheckRD = true;
                        } else {  // if ft_blockVar > 28.3802
                            skipCheckRD = true;
                        }
                    } else {  // if ft_blockVar > 65.2714
                        if (ft_qp <= 34.5000) {
                            if (ft_height <= 780.0000) {
                                skipCheckRD = false;
                            } else {  // if ft_height > 780.0000
                                if (ft_diffVar <= 78.8807) {
                                    if (ft_height <= 1620.0000) {
                                        skipCheckRD = false;
                                    } else {  // if ft_height > 1620.0000
                                        skipCheckRD = true;
                                    }
                                } else {  // if ft_diffVar > 78.8807
                                    skipCheckRD = false;
                                }
                            }
                        } else {  // if ft_qp > 34.5000
                            if (ft_diffVar <= 110.5894) {
                                if (ft_height <= 780.0000) {
                                    skipCheckRD = false;
                                } else {  // if ft_height > 780.0000
                                    skipCheckRD = true;
                                }
                            } else {  // if ft_diffVar > 110.5894
                                skipCheckRD = false;
                            }
                        }
                    }
                }
            }
        }
    } else {  // if ft_blockVar > 252.3939
        if (ft_qp <= 29.5000) {
            if (ft_qp <= 24.5000) {
                if (ft_diffVar <= 4.8890) {
                    if (ft_config <= 0.5000) {
                        if (ft_height <= 780.0000) {
                            skipCheckRD = false;
                        } else {  // if ft_height > 780.0000
                            if (ft_diffVar <= 3.1891) {
                                skipCheckRD = true;
                            } else {  // if ft_diffVar > 3.1891
                                if (ft_blockVar <= 585.8572) {
                                    skipCheckRD = true;
                                } else {  // if ft_blockVar > 585.8572
                                    skipCheckRD = false;
                                }
                            }
                        }
                    } else {  // if ft_config > 0.5000
                        if (ft_diffVar <= 3.3911) {
                            if (ft_diffVar <= 1.4481) {
                                skipCheckRD = true;
                            } else {  // if ft_diffVar > 1.4481
                                if (ft_height <= 780.0000) {
                                    skipCheckRD = false;
                                } else {  // if ft_height > 780.0000
                                    skipCheckRD = true;
                                }
                            }
                        } else {  // if ft_diffVar > 3.3911
                            skipCheckRD = false;
                        }
                    }
                } else {  // if ft_diffVar > 4.8890
                    if (ft_config <= 0.5000) {
                        if (ft_diffVar <= 172.3579) {
                            if (ft_height <= 1620.0000) {
                                if (ft_blockVar <= 313.3019) {
                                    skipCheckRD = false;
                                } else {  // if ft_blockVar > 313.3019
                                    skipCheckRD = false;
                                }
                            } else {  // if ft_height > 1620.0000
                                if (ft_diffVar <= 23.6575) {
                                    if (ft_blockVar <= 371.1039) {
                                        skipCheckRD = false;
                                    } else {  // if ft_blockVar > 371.1039
                                        skipCheckRD = false;
                                    }
                                } else {  // if ft_diffVar > 23.6575
                                    if (ft_blockVar <= 746.8441) {
                                        skipCheckRD = false;
                                    } else {  // if ft_blockVar > 746.8441
                                        skipCheckRD = false;
                                    }
                                }
                            }
                        } else {  // if ft_diffVar > 172.3579
                            if (ft_height <= 1620.0000) {
                                if (ft_blockVar <= 540.3089) {
                                    skipCheckRD = false;
                                } else {  // if ft_blockVar > 540.3089
                                    skipCheckRD = false;
                                }
                            } else {  // if ft_height > 1620.0000
                                if (ft_blockVar <= 2414.2539) {
                                    if (ft_diffVar <= 269.3652) {
                                        skipCheckRD = false;
                                    } else {  // if ft_diffVar > 269.3652
                                        skipCheckRD = false;
                                    }
                                } else {  // if ft_blockVar > 2414.2539
                                    skipCheckRD = false;
                                }
                            }
                        }
                    } else {  // if ft_config > 0.5000
                        if (ft_height <= 1620.0000) {
                            skipCheckRD = false;
                        } else {  // if ft_height > 1620.0000
                            if (ft_diffVar <= 36.2956) {
                                skipCheckRD = false;
                            } else {  // if ft_diffVar > 36.2956
                                skipCheckRD = false;
                            }
                        }
                    }
                }
            } else {  // if ft_qp > 24.5000
                if (ft_diffVar <= 7.6143) {
                    if (ft_diffVar <= 5.1998) {
                        if (ft_height <= 780.0000) {
                            skipCheckRD = true;
                        } else {  // if ft_height > 780.0000
                            skipCheckRD = true;
                        }
                    } else {  // if ft_diffVar > 5.1998
                        if (ft_config <= 0.5000) {
                            skipCheckRD = true;
                        } else {  // if ft_config > 0.5000
                            if (ft_height <= 780.0000) {
                                skipCheckRD = false;
                            } else {  // if ft_height > 780.0000
                                skipCheckRD = false;
                            }
                        }
                    }
                } else {  // if ft_diffVar > 7.6143
                    if (ft_diffVar <= 108.1304) {
                        if (ft_config <= 0.5000) {
                            if (ft_blockVar <= 648.1491) {
                                if (ft_height <= 1620.0000) {
                                    if (ft_diffVar <= 14.3214) {
                                        skipCheckRD = true;
                                    } else {  // if ft_diffVar > 14.3214
                                        skipCheckRD = false;
                                    }
                                } else {  // if ft_height > 1620.0000
                                    if (ft_diffVar <= 62.4692) {
                                        skipCheckRD = true;
                                    } else {  // if ft_diffVar > 62.4692
                                        skipCheckRD = false;
                                    }
                                }
                            } else {  // if ft_blockVar > 648.1491
                                if (ft_diffVar <= 37.0339) {
                                    if (ft_height <= 1620.0000) {
                                        skipCheckRD = false;
                                    } else {  // if ft_height > 1620.0000
                                        skipCheckRD = true;
                                    }
                                } else {  // if ft_diffVar > 37.0339
                                    skipCheckRD = false;
                                }
                            }
                        } else {  // if ft_config > 0.5000
                            if (ft_blockVar <= 774.3961) {
                                if (ft_height <= 1620.0000) {
                                    skipCheckRD = false;
                                } else {  // if ft_height > 1620.0000
                                    if (ft_diffVar <= 23.9503) {
                                        skipCheckRD = false;
                                    } else {  // if ft_diffVar > 23.9503
                                        if (ft_blockVar <= 458.9805) {
                                            skipCheckRD = false;
                                        } else {  // if ft_blockVar > 458.9805
                                            skipCheckRD = false;
                                        }
                                    }
                                }
                            } else {  // if ft_blockVar > 774.3961
                                skipCheckRD = false;
                            }
                        }
                    } else {  // if ft_diffVar > 108.1304
                        if (ft_diffVar <= 340.1135) {
                            if (ft_config <= 0.5000) {
                                if (ft_height <= 780.0000) {
                                    skipCheckRD = false;
                                } else {  // if ft_height > 780.0000
                                    if (ft_blockVar <= 545.8355) {
                                        if (ft_diffVar <= 202.8913) {
                                            skipCheckRD = false;
                                        } else {  // if ft_diffVar > 202.8913
                                            skipCheckRD = false;
                                        }
                                    } else {  // if ft_blockVar > 545.8355
                                        skipCheckRD = false;
                                    }
                                }
                            } else {  // if ft_config > 0.5000
                                if (ft_blockVar <= 481.9578) {
                                    skipCheckRD = false;
                                } else {  // if ft_blockVar > 481.9578
                                    skipCheckRD = false;
                                }
                            }
                        } else {  // if ft_diffVar > 340.1135
                            if (ft_height <= 1620.0000) {
                                skipCheckRD = false;
                            } else {  // if ft_height > 1620.0000
                                if (ft_blockVar <= 2168.9250) {
                                    skipCheckRD = false;
                                } else {  // if ft_blockVar > 2168.9250
                                    skipCheckRD = false;
                                }
                            }
                        }
                    }
                }
            }
        } else {  // if ft_qp > 29.5000
            if (ft_diffVar <= 43.4114) {
                if (ft_diffVar <= 19.2070) {
                    if (ft_diffVar <= 7.0528) {
                        skipCheckRD = true;
                    } else {  // if ft_diffVar > 7.0528
                        if (ft_qp <= 34.5000) {
                            if (ft_config <= 0.5000) {
                                skipCheckRD = true;
                            } else {  // if ft_config > 0.5000
                                if (ft_diffVar <= 16.6690) {
                                    skipCheckRD = true;
                                } else {  // if ft_diffVar > 16.6690
                                    if (ft_blockVar <= 293.7707) {
                                        skipCheckRD = true;
                                    } else {  // if ft_blockVar > 293.7707
                                        skipCheckRD = false;
                                    }
                                }
                            }
                        } else {  // if ft_qp > 34.5000
                            if (ft_height <= 1620.0000) {
                                skipCheckRD = true;
                            } else {  // if ft_height > 1620.0000
                                if (ft_config <= 0.5000) {
                                    skipCheckRD = true;
                                } else {  // if ft_config > 0.5000
                                    skipCheckRD = true;
                                }
                            }
                        }
                    }
                } else {  // if ft_diffVar > 19.2070
                    if (ft_config <= 0.5000) {
                        if (ft_qp <= 34.5000) {
                            if (ft_height <= 1620.0000) {
                                skipCheckRD = false;
                            } else {  // if ft_height > 1620.0000
                                skipCheckRD = true;
                            }
                        } else {  // if ft_qp > 34.5000
                            skipCheckRD = true;
                        }
                    } else {  // if ft_config > 0.5000
                        if (ft_qp <= 34.5000) {
                            if (ft_blockVar <= 650.9212) {
                                if (ft_height <= 1620.0000) {
                                    skipCheckRD = false;
                                } else {  // if ft_height > 1620.0000
                                    skipCheckRD = false;
                                }
                            } else {  // if ft_blockVar > 650.9212
                                skipCheckRD = false;
                            }
                        } else {  // if ft_qp > 34.5000
                            if (ft_blockVar <= 801.5486) {
                                skipCheckRD = true;
                            } else {  // if ft_blockVar > 801.5486
                                skipCheckRD = false;
                            }
                        }
                    }
                }
            } else {  // if ft_diffVar > 43.4114
                if (ft_blockVar <= 504.4937) {
                    if (ft_diffVar <= 115.6720) {
                        if (ft_height <= 1620.0000) {
                            if (ft_config <= 0.5000) {
                                if (ft_diffVar <= 83.3290) {
                                    if (ft_qp <= 34.5000) {
                                        skipCheckRD = false;
                                    } else {  // if ft_qp > 34.5000
                                        skipCheckRD = true;
                                    }
                                } else {  // if ft_diffVar > 83.3290
                                    skipCheckRD = false;
                                }
                            } else {  // if ft_config > 0.5000
                                skipCheckRD = false;
                            }
                        } else {  // if ft_height > 1620.0000
                            if (ft_qp <= 34.5000) {
                                if (ft_config <= 0.5000) {
                                    if (ft_blockVar <= 318.0191) {
                                        skipCheckRD = false;
                                    } else {  // if ft_blockVar > 318.0191
                                        skipCheckRD = true;
                                    }
                                } else {  // if ft_config > 0.5000
                                    skipCheckRD = false;
                                }
                            } else {  // if ft_qp > 34.5000
                                if (ft_diffVar <= 79.3440) {
                                    skipCheckRD = true;
                                } else {  // if ft_diffVar > 79.3440
                                    skipCheckRD = true;
                                }
                            }
                        }
                    } else {  // if ft_diffVar > 115.6720
                        if (ft_qp <= 34.5000) {
                            if (ft_height <= 780.0000) {
                                skipCheckRD = false;
                            } else {  // if ft_height > 780.0000
                                skipCheckRD = false;
                            }
                        } else {  // if ft_qp > 34.5000
                            if (ft_height <= 780.0000) {
                                skipCheckRD = false;
                            } else {  // if ft_height > 780.0000
                                if (ft_diffVar <= 318.6821) {
                                    if (ft_config <= 0.5000) {
                                        if (ft_diffVar <= 195.0162) {
                                            skipCheckRD = true;
                                        } else {  // if ft_diffVar > 195.0162
                                            skipCheckRD = false;
                                        }
                                    } else {  // if ft_config > 0.5000
                                        skipCheckRD = false;
                                    }
                                } else {  // if ft_diffVar > 318.6821
                                    skipCheckRD = false;
                                }
                            }
                        }
                    }
                } else {  // if ft_blockVar > 504.4937
                    if (ft_height <= 1620.0000) {
                        if (ft_diffVar <= 129.8763) {
                            if (ft_config <= 0.5000) {
                                if (ft_diffVar <= 71.9624) {
                                    skipCheckRD = false;
                                } else {  // if ft_diffVar > 71.9624
                                    if (ft_blockVar <= 898.7229) {
                                        skipCheckRD = false;
                                    } else {  // if ft_blockVar > 898.7229
                                        if (ft_blockVar <= 1678.5248) {
                                            skipCheckRD = false;
                                        } else {  // if ft_blockVar > 1678.5248
                                            skipCheckRD = false;
                                        }
                                    }
                                }
                            } else {  // if ft_config > 0.5000
                                if (ft_qp <= 34.5000) {
                                    skipCheckRD = false;
                                } else {  // if ft_qp > 34.5000
                                    if (ft_diffVar <= 73.5519) {
                                        skipCheckRD = false;
                                    } else {  // if ft_diffVar > 73.5519
                                        if (ft_blockVar <= 898.7229) {
                                            skipCheckRD = false;
                                        } else {  // if ft_blockVar > 898.7229
                                            if (ft_blockVar <= 970.0853) {
                                                skipCheckRD = true;
                                            } else {  // if ft_blockVar > 970.0853
                                                skipCheckRD = false;
                                            }
                                        }
                                    }
                                }
                            }
                        } else {  // if ft_diffVar > 129.8763
                            if (ft_height <= 780.0000) {
                                skipCheckRD = false;
                            } else {  // if ft_height > 780.0000
                                if (ft_qp <= 34.5000) {
                                    skipCheckRD = false;
                                } else {  // if ft_qp > 34.5000
                                    skipCheckRD = false;
                                }
                            }
                        }
                    } else {  // if ft_height > 1620.0000
                        if (ft_qp <= 34.5000) {
                            if (ft_config <= 0.5000) {
                                if (ft_diffVar <= 220.2053) {
                                    if (ft_diffVar <= 107.3744) {
                                        if (ft_blockVar <= 688.6369) {
                                            skipCheckRD = true;
                                        } else {  // if ft_blockVar > 688.6369
                                            skipCheckRD = false;
                                        }
                                    } else {  // if ft_diffVar > 107.3744
                                        if (ft_blockVar <= 1784.7091) {
                                            skipCheckRD = false;
                                        } else {  // if ft_blockVar > 1784.7091
                                            skipCheckRD = false;
                                        }
                                    }
                                } else {  // if ft_diffVar > 220.2053
                                    skipCheckRD = false;
                                }
                            } else {  // if ft_config > 0.5000
                                skipCheckRD = false;
                            }
                        } else {  // if ft_qp > 34.5000
                            if (ft_diffVar <= 185.7704) {
                                if (ft_config <= 0.5000) {
                                    if (ft_diffVar <= 59.7157) {
                                        skipCheckRD = true;
                                    } else {  // if ft_diffVar > 59.7157
                                        skipCheckRD = false;
                                    }
                                } else {  // if ft_config > 0.5000
                                    if (ft_blockVar <= 1622.0334) {
                                        if (ft_diffVar <= 83.7474) {
                                            skipCheckRD = false;
                                        } else {  // if ft_diffVar > 83.7474
                                            skipCheckRD = false;
                                        }
                                    } else {  // if ft_blockVar > 1622.0334
                                        skipCheckRD = false;
                                    }
                                }
                            } else {  // if ft_diffVar > 185.7704
                                if (ft_diffVar <= 321.0325) {
                                    if (ft_config <= 0.5000) {
                                        skipCheckRD = false;
                                    } else {  // if ft_config > 0.5000
                                        skipCheckRD = false;
                                    }
                                } else {  // if ft_diffVar > 321.0325
                                    skipCheckRD = false;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
            break;
        case 1:
            if (ft_previousSplit <= 0.5000) {
                if (ft_blockVar <= 126.9821) {
                    if (ft_blockVar <= 38.9473) {
                        if (ft_qp <= 24.5000) {
                            if (ft_blockVar <= 15.5980) {
                                if (ft_blockVar <= 12.3528) {
                                    skipCheckRD = true;
                                } else {  // if ft_blockVar > 12.3528
                                    skipCheckRD = true;
                                }
                            } else {  // if ft_blockVar > 15.5980
                                if (ft_diffVar <= 5.0356) {
                                    if (ft_config <= 0.5000) {
                                        skipCheckRD = true;
                                    } else {  // if ft_config > 0.5000
                                        skipCheckRD = true;
                                    }
                                } else {  // if ft_diffVar > 5.0356
                                    if (ft_height <= 1620.0000) {
                                        skipCheckRD = true;
                                    } else {  // if ft_height > 1620.0000
                                        if (ft_config <= 0.5000) {
                                            skipCheckRD = true;
                                        } else {  // if ft_config > 0.5000
                                            if (ft_blockVar <= 20.8140) {
                                                skipCheckRD = true;
                                            } else {  // if ft_blockVar > 20.8140
                                                skipCheckRD = true;
                                            }
                                        }
                                    }
                                }
                            }
                        } else {  // if ft_qp > 24.5000
                            if (ft_diffVar <= 5.4548) {
                                skipCheckRD = true;
                            } else {  // if ft_diffVar > 5.4548
                                if (ft_qp <= 29.5000) {
                                    if (ft_blockVar <= 22.1030) {
                                        skipCheckRD = true;
                                    } else {  // if ft_blockVar > 22.1030
                                        skipCheckRD = true;
                                    }
                                } else {  // if ft_qp > 29.5000
                                    skipCheckRD = true;
                                }
                            }
                        }
                    } else {  // if ft_blockVar > 38.9473
                        if (ft_qp <= 24.5000) {
                            if (ft_diffVar <= 17.6951) {
                                if (ft_config <= 0.5000) {
                                    if (ft_diffVar <= 5.6833) {
                                        skipCheckRD = true;
                                    } else {  // if ft_diffVar > 5.6833
                                        skipCheckRD = true;
                                    }
                                } else {  // if ft_config > 0.5000
                                    if (ft_diffVar <= 3.9682) {
                                        skipCheckRD = true;
                                    } else {  // if ft_diffVar > 3.9682
                                        if (ft_blockVar <= 60.1397) {
                                            if (ft_height <= 1620.0000) {
                                                if (ft_diffVar <= 6.2045) {
                                                    skipCheckRD = true;
                                                } else {  // if ft_diffVar > 6.2045
                                                    skipCheckRD = true;
                                                }
                                            } else {  // if ft_height > 1620.0000
                                                if (ft_diffVar <= 6.7392) {
                                                    skipCheckRD = true;
                                                } else {  // if ft_diffVar > 6.7392
                                                    skipCheckRD = true;
                                                }
                                            }
                                        } else {  // if ft_blockVar > 60.1397
                                            if (ft_height <= 1620.0000) {
                                                if (ft_diffVar <= 10.5931) {
                                                    skipCheckRD = false;
                                                } else {  // if ft_diffVar > 10.5931
                                                    skipCheckRD = true;
                                                }
                                            } else {  // if ft_height > 1620.0000
                                                skipCheckRD = true;
                                            }
                                        }
                                    }
                                }
                            } else {  // if ft_diffVar > 17.6951
                                if (ft_config <= 0.5000) {
                                    if (ft_height <= 780.0000) {
                                        skipCheckRD = false;
                                    } else {  // if ft_height > 780.0000
                                        if (ft_diffVar <= 94.7784) {
                                            skipCheckRD = true;
                                        } else {  // if ft_diffVar > 94.7784
                                            skipCheckRD = true;
                                        }
                                    }
                                } else {  // if ft_config > 0.5000
                                    if (ft_height <= 1620.0000) {
                                        if (ft_blockVar <= 71.4075) {
                                            skipCheckRD = true;
                                        } else {  // if ft_blockVar > 71.4075
                                            skipCheckRD = false;
                                        }
                                    } else {  // if ft_height > 1620.0000
                                        if (ft_diffVar <= 593.3209) {
                                            if (ft_diffVar <= 56.3946) {
                                                if (ft_diffVar <= 24.4411) {
                                                    skipCheckRD = false;
                                                } else {  // if ft_diffVar > 24.4411
                                                    if (ft_blockVar <= 52.1008) {
                                                        skipCheckRD = true;
                                                    } else {  // if ft_blockVar > 52.1008
                                                        skipCheckRD = false;
                                                    }
                                                }
                                            } else {  // if ft_diffVar > 56.3946
                                                if (ft_blockVar <= 98.3060) {
                                                    skipCheckRD = true;
                                                } else {  // if ft_blockVar > 98.3060
                                                    skipCheckRD = false;
                                                }
                                            }
                                        } else {  // if ft_diffVar > 593.3209
                                            skipCheckRD = true;
                                        }
                                    }
                                }
                            }
                        } else {  // if ft_qp > 24.5000
                            if (ft_diffVar <= 13.0428) {
                                if (ft_diffVar <= 7.9536) {
                                    skipCheckRD = true;
                                } else {  // if ft_diffVar > 7.9536
                                    if (ft_qp <= 29.5000) {
                                        skipCheckRD = true;
                                    } else {  // if ft_qp > 29.5000
                                        skipCheckRD = true;
                                    }
                                }
                            } else {  // if ft_diffVar > 13.0428
                                if (ft_qp <= 29.5000) {
                                    if (ft_blockVar <= 68.9562) {
                                        if (ft_height <= 780.0000) {
                                            skipCheckRD = false;
                                        } else {  // if ft_height > 780.0000
                                            skipCheckRD = true;
                                        }
                                    } else {  // if ft_blockVar > 68.9562
                                        skipCheckRD = true;
                                    }
                                } else {  // if ft_qp > 29.5000
                                    if (ft_qp <= 34.5000) {
                                        if (ft_height <= 780.0000) {
                                            skipCheckRD = true;
                                        } else {  // if ft_height > 780.0000
                                            if (ft_blockVar <= 82.5732) {
                                                skipCheckRD = true;
                                            } else {  // if ft_blockVar > 82.5732
                                                skipCheckRD = true;
                                            }
                                        }
                                    } else {  // if ft_qp > 34.5000
                                        if (ft_height <= 780.0000) {
                                            skipCheckRD = true;
                                        } else {  // if ft_height > 780.0000
                                            skipCheckRD = true;
                                        }
                                    }
                                }
                            }
                        }
                    }
                } else {  // if ft_blockVar > 126.9821
                    if (ft_qp <= 24.5000) {
                        if (ft_config <= 0.5000) {
                            if (ft_diffVar <= 73.4685) {
                                if (ft_diffVar <= 6.5776) {
                                    if (ft_blockVar <= 497.6309) {
                                        skipCheckRD = true;
                                    } else {  // if ft_blockVar > 497.6309
                                        if (ft_diffVar <= 4.7319) {
                                            skipCheckRD = true;
                                        } else {  // if ft_diffVar > 4.7319
                                            skipCheckRD = true;
                                        }
                                    }
                                } else {  // if ft_diffVar > 6.5776
                                    if (ft_height <= 780.0000) {
                                        skipCheckRD = false;
                                    } else {  // if ft_height > 780.0000
                                        skipCheckRD = true;
                                    }
                                }
                            } else {  // if ft_diffVar > 73.4685
                                if (ft_diffVar <= 321.8731) {
                                    if (ft_height <= 780.0000) {
                                        skipCheckRD = false;
                                    } else {  // if ft_height > 780.0000
                                        if (ft_diffVar <= 225.4820) {
                                            if (ft_blockVar <= 299.5108) {
                                                skipCheckRD = true;
                                            } else {  // if ft_blockVar > 299.5108
                                                if (ft_blockVar <= 1344.3445) {
                                                    skipCheckRD = true;
                                                } else {  // if ft_blockVar > 1344.3445
                                                    skipCheckRD = true;
                                                }
                                            }
                                        } else {  // if ft_diffVar > 225.4820
                                            skipCheckRD = true;
                                        }
                                    }
                                } else {  // if ft_diffVar > 321.8731
                                    if (ft_height <= 1620.0000) {
                                        if (ft_diffVar <= 819.5551) {
                                            skipCheckRD = false;
                                        } else {  // if ft_diffVar > 819.5551
                                            skipCheckRD = false;
                                        }
                                    } else {  // if ft_height > 1620.0000
                                        if (ft_blockVar <= 515.4392) {
                                            skipCheckRD = true;
                                        } else {  // if ft_blockVar > 515.4392
                                            if (ft_blockVar <= 1689.7532) {
                                                if (ft_diffVar <= 436.3133) {
                                                    skipCheckRD = true;
                                                } else {  // if ft_diffVar > 436.3133
                                                    skipCheckRD = false;
                                                }
                                            } else {  // if ft_blockVar > 1689.7532
                                                skipCheckRD = true;
                                            }
                                        }
                                    }
                                }
                            }
                        } else {  // if ft_config > 0.5000
                            if (ft_diffVar <= 27.4643) {
                                if (ft_blockVar <= 833.8369) {
                                    if (ft_diffVar <= 3.3583) {
                                        skipCheckRD = true;
                                    } else {  // if ft_diffVar > 3.3583
                                        if (ft_height <= 1620.0000) {
                                            if (ft_diffVar <= 9.1953) {
                                                if (ft_diffVar <= 4.6895) {
                                                    skipCheckRD = false;
                                                } else {  // if ft_diffVar > 4.6895
                                                    skipCheckRD = false;
                                                }
                                            } else {  // if ft_diffVar > 9.1953
                                                skipCheckRD = false;
                                            }
                                        } else {  // if ft_height > 1620.0000
                                            if (ft_diffVar <= 5.1309) {
                                                skipCheckRD = true;
                                            } else {  // if ft_diffVar > 5.1309
                                                skipCheckRD = true;
                                            }
                                        }
                                    }
                                } else {  // if ft_blockVar > 833.8369
                                    if (ft_diffVar <= 4.1122) {
                                        if (ft_diffVar <= 3.3241) {
                                            skipCheckRD = true;
                                        } else {  // if ft_diffVar > 3.3241
                                            skipCheckRD = false;
                                        }
                                    } else {  // if ft_diffVar > 4.1122
                                        if (ft_height <= 1620.0000) {
                                            skipCheckRD = false;
                                        } else {  // if ft_height > 1620.0000
                                            skipCheckRD = false;
                                        }
                                    }
                                }
                            } else {  // if ft_diffVar > 27.4643
                                if (ft_blockVar <= 266.1473) {
                                    if (ft_diffVar <= 313.0743) {
                                        skipCheckRD = false;
                                    } else {  // if ft_diffVar > 313.0743
                                        skipCheckRD = true;
                                    }
                                } else {  // if ft_blockVar > 266.1473
                                    if (ft_blockVar <= 2156.4554) {
                                        if (ft_diffVar <= 57.3785) {
                                            skipCheckRD = false;
                                        } else {  // if ft_diffVar > 57.3785
                                            if (ft_height <= 780.0000) {
                                                skipCheckRD = false;
                                            } else {  // if ft_height > 780.0000
                                                if (ft_diffVar <= 1057.7679) {
                                                    if (ft_diffVar <= 295.0262) {
                                                        if (ft_height <= 1620.0000) {
                                                            if (ft_blockVar <= 377.7243) {
                                                                if (ft_diffVar <= 185.3305) {
                                                                    skipCheckRD = false;
                                                                } else {  // if ft_diffVar > 185.3305
                                                                    skipCheckRD = true;
                                                                }
                                                            } else {  // if ft_blockVar > 377.7243
                                                                skipCheckRD = false;
                                                            }
                                                        } else {  // if ft_height > 1620.0000
                                                            skipCheckRD = false;
                                                        }
                                                    } else {  // if ft_diffVar > 295.0262
                                                        if (ft_height <= 1620.0000) {
                                                            if (ft_blockVar <= 553.2232) {
                                                                skipCheckRD = false;
                                                            } else {  // if ft_blockVar > 553.2232
                                                                skipCheckRD = false;
                                                            }
                                                        } else {  // if ft_height > 1620.0000
                                                            if (ft_diffVar <= 578.4290) {
                                                                skipCheckRD = false;
                                                            } else {  // if ft_diffVar > 578.4290
                                                                skipCheckRD = false;
                                                            }
                                                        }
                                                    }
                                                } else {  // if ft_diffVar > 1057.7679
                                                    skipCheckRD = false;
                                                }
                                            }
                                        }
                                    } else {  // if ft_blockVar > 2156.4554
                                        if (ft_height <= 1620.0000) {
                                            if (ft_diffVar <= 300.5915) {
                                                skipCheckRD = false;
                                            } else {  // if ft_diffVar > 300.5915
                                                skipCheckRD = false;
                                            }
                                        } else {  // if ft_height > 1620.0000
                                            if (ft_diffVar <= 280.3468) {
                                                skipCheckRD = false;
                                            } else {  // if ft_diffVar > 280.3468
                                                skipCheckRD = true;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    } else {  // if ft_qp > 24.5000
                        if (ft_diffVar <= 30.5403) {
                            if (ft_diffVar <= 7.4247) {
                                if (ft_qp <= 29.5000) {
                                    if (ft_diffVar <= 6.8153) {
                                        skipCheckRD = true;
                                    } else {  // if ft_diffVar > 6.8153
                                        skipCheckRD = true;
                                    }
                                } else {  // if ft_qp > 29.5000
                                    skipCheckRD = true;
                                }
                            } else {  // if ft_diffVar > 7.4247
                                if (ft_qp <= 29.5000) {
                                    if (ft_height <= 1620.0000) {
                                        if (ft_config <= 0.5000) {
                                            skipCheckRD = true;
                                        } else {  // if ft_config > 0.5000
                                            if (ft_blockVar <= 485.7702) {
                                                skipCheckRD = true;
                                            } else {  // if ft_blockVar > 485.7702
                                                if (ft_diffVar <= 7.9366) {
                                                    skipCheckRD = true;
                                                } else {  // if ft_diffVar > 7.9366
                                                    skipCheckRD = false;
                                                }
                                            }
                                        }
                                    } else {  // if ft_height > 1620.0000
                                        if (ft_diffVar <= 14.5100) {
                                            skipCheckRD = true;
                                        } else {  // if ft_diffVar > 14.5100
                                            if (ft_config <= 0.5000) {
                                                skipCheckRD = true;
                                            } else {  // if ft_config > 0.5000
                                                if (ft_blockVar <= 1073.9278) {
                                                    skipCheckRD = true;
                                                } else {  // if ft_blockVar > 1073.9278
                                                    skipCheckRD = false;
                                                }
                                            }
                                        }
                                    }
                                } else {  // if ft_qp > 29.5000
                                    if (ft_qp <= 34.5000) {
                                        if (ft_diffVar <= 12.6723) {
                                            skipCheckRD = true;
                                        } else {  // if ft_diffVar > 12.6723
                                            if (ft_height <= 1620.0000) {
                                                if (ft_diffVar <= 20.2547) {
                                                    skipCheckRD = true;
                                                } else {  // if ft_diffVar > 20.2547
                                                    skipCheckRD = true;
                                                }
                                            } else {  // if ft_height > 1620.0000
                                                if (ft_diffVar <= 18.4967) {
                                                    skipCheckRD = true;
                                                } else {  // if ft_diffVar > 18.4967
                                                    skipCheckRD = true;
                                                }
                                            }
                                        }
                                    } else {  // if ft_qp > 34.5000
                                        skipCheckRD = true;
                                    }
                                }
                            }
                        } else {  // if ft_diffVar > 30.5403
                            if (ft_height <= 1620.0000) {
                                if (ft_height <= 780.0000) {
                                    if (ft_qp <= 34.5000) {
                                        if (ft_diffVar <= 111.3831) {
                                            skipCheckRD = false;
                                        } else {  // if ft_diffVar > 111.3831
                                            skipCheckRD = false;
                                        }
                                    } else {  // if ft_qp > 34.5000
                                        if (ft_diffVar <= 138.2275) {
                                            if (ft_blockVar <= 490.4136) {
                                                if (ft_blockVar <= 245.5288) {
                                                    skipCheckRD = true;
                                                } else {  // if ft_blockVar > 245.5288
                                                    skipCheckRD = false;
                                                }
                                            } else {  // if ft_blockVar > 490.4136
                                                skipCheckRD = true;
                                            }
                                        } else {  // if ft_diffVar > 138.2275
                                            skipCheckRD = false;
                                        }
                                    }
                                } else {  // if ft_height > 780.0000
                                    if (ft_blockVar <= 508.8434) {
                                        if (ft_qp <= 34.5000) {
                                            if (ft_config <= 0.5000) {
                                                if (ft_diffVar <= 93.1764) {
                                                    skipCheckRD = true;
                                                } else {  // if ft_diffVar > 93.1764
                                                    skipCheckRD = true;
                                                }
                                            } else {  // if ft_config > 0.5000
                                                skipCheckRD = true;
                                            }
                                        } else {  // if ft_qp > 34.5000
                                            if (ft_blockVar <= 192.5855) {
                                                skipCheckRD = true;
                                            } else {  // if ft_blockVar > 192.5855
                                                skipCheckRD = true;
                                            }
                                        }
                                    } else {  // if ft_blockVar > 508.8434
                                        if (ft_diffVar <= 645.7973) {
                                            if (ft_config <= 0.5000) {
                                                if (ft_blockVar <= 1708.3298) {
                                                    if (ft_diffVar <= 90.5604) {
                                                        skipCheckRD = true;
                                                    } else {  // if ft_diffVar > 90.5604
                                                        skipCheckRD = true;
                                                    }
                                                } else {  // if ft_blockVar > 1708.3298
                                                    skipCheckRD = true;
                                                }
                                            } else {  // if ft_config > 0.5000
                                                if (ft_qp <= 34.5000) {
                                                    skipCheckRD = false;
                                                } else {  // if ft_qp > 34.5000
                                                    if (ft_diffVar <= 41.9821) {
                                                        skipCheckRD = true;
                                                    } else {  // if ft_diffVar > 41.9821
                                                        skipCheckRD = true;
                                                    }
                                                }
                                            }
                                        } else {  // if ft_diffVar > 645.7973
                                            if (ft_config <= 0.5000) {
                                                if (ft_diffVar <= 1476.7183) {
                                                    skipCheckRD = false;
                                                } else {  // if ft_diffVar > 1476.7183
                                                    skipCheckRD = false;
                                                }
                                            } else {  // if ft_config > 0.5000
                                                if (ft_qp <= 29.5000) {
                                                    skipCheckRD = false;
                                                } else {  // if ft_qp > 29.5000
                                                    skipCheckRD = false;
                                                }
                                            }
                                        }
                                    }
                                }
                            } else {  // if ft_height > 1620.0000
                                if (ft_qp <= 29.5000) {
                                    if (ft_blockVar <= 276.8362) {
                                        if (ft_diffVar <= 86.6754) {
                                            skipCheckRD = true;
                                        } else {  // if ft_diffVar > 86.6754
                                            skipCheckRD = true;
                                        }
                                    } else {  // if ft_blockVar > 276.8362
                                        if (ft_blockVar <= 839.6302) {
                                            if (ft_diffVar <= 49.3037) {
                                                skipCheckRD = true;
                                            } else {  // if ft_diffVar > 49.3037
                                                if (ft_config <= 0.5000) {
                                                    if (ft_diffVar <= 110.5332) {
                                                        skipCheckRD = true;
                                                    } else {  // if ft_diffVar > 110.5332
                                                        if (ft_diffVar <= 510.8671) {
                                                            skipCheckRD = true;
                                                        } else {  // if ft_diffVar > 510.8671
                                                            skipCheckRD = false;
                                                        }
                                                    }
                                                } else {  // if ft_config > 0.5000
                                                    if (ft_diffVar <= 770.0569) {
                                                        skipCheckRD = true;
                                                    } else {  // if ft_diffVar > 770.0569
                                                        skipCheckRD = true;
                                                    }
                                                }
                                            }
                                        } else {  // if ft_blockVar > 839.6302
                                            if (ft_config <= 0.5000) {
                                                if (ft_diffVar <= 486.0368) {
                                                    skipCheckRD = true;
                                                } else {  // if ft_diffVar > 486.0368
                                                    skipCheckRD = false;
                                                }
                                            } else {  // if ft_config > 0.5000
                                                skipCheckRD = false;
                                            }
                                        }
                                    }
                                } else {  // if ft_qp > 29.5000
                                    if (ft_blockVar <= 281.7244) {
                                        if (ft_qp <= 34.5000) {
                                            if (ft_blockVar <= 212.2771) {
                                                skipCheckRD = true;
                                            } else {  // if ft_blockVar > 212.2771
                                                if (ft_diffVar <= 135.1969) {
                                                    skipCheckRD = true;
                                                } else {  // if ft_diffVar > 135.1969
                                                    skipCheckRD = true;
                                                }
                                            }
                                        } else {  // if ft_qp > 34.5000
                                            if (ft_diffVar <= 64.6905) {
                                                skipCheckRD = true;
                                            } else {  // if ft_diffVar > 64.6905
                                                skipCheckRD = true;
                                            }
                                        }
                                    } else {  // if ft_blockVar > 281.7244
                                        if (ft_qp <= 34.5000) {
                                            if (ft_blockVar <= 834.9213) {
                                                if (ft_diffVar <= 90.2370) {
                                                    if (ft_config <= 0.5000) {
                                                        skipCheckRD = true;
                                                    } else {  // if ft_config > 0.5000
                                                        skipCheckRD = true;
                                                    }
                                                } else {  // if ft_diffVar > 90.2370
                                                    skipCheckRD = true;
                                                }
                                            } else {  // if ft_blockVar > 834.9213
                                                skipCheckRD = true;
                                            }
                                        } else {  // if ft_qp > 34.5000
                                            if (ft_blockVar <= 836.0855) {
                                                if (ft_diffVar <= 72.1997) {
                                                    skipCheckRD = true;
                                                } else {  // if ft_diffVar > 72.1997
                                                    skipCheckRD = true;
                                                }
                                            } else {  // if ft_blockVar > 836.0855
                                                if (ft_blockVar <= 2623.1687) {
                                                    if (ft_diffVar <= 52.8449) {
                                                        skipCheckRD = true;
                                                    } else {  // if ft_diffVar > 52.8449
                                                        skipCheckRD = true;
                                                    }
                                                } else {  // if ft_blockVar > 2623.1687
                                                    skipCheckRD = true;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            } else {  // if ft_previousSplit > 0.5000
    if (ft_blockVar <= 96.8052) {
        if (ft_blockVar <= 33.9940) {
            if (ft_blockVar <= 21.2116) {
                if (ft_qp <= 24.5000) {
                    if (ft_blockVar <= 13.4776) {
                        skipCheckRD = true;
                    } else {  // if ft_blockVar > 13.4776
                        skipCheckRD = true;
                    }
                } else {  // if ft_qp > 24.5000
                    skipCheckRD = true;
                }
            } else {  // if ft_blockVar > 21.2116
                if (ft_qp <= 24.5000) {
                    if (ft_diffVar <= 9.8758) {
                        skipCheckRD = true;
                    } else {  // if ft_diffVar > 9.8758
                        skipCheckRD = true;
                    }
                } else {  // if ft_qp > 24.5000
                    skipCheckRD = true;
                }
            }
        } else {  // if ft_blockVar > 33.9940
            if (ft_qp <= 24.5000) {
                if (ft_diffVar <= 14.9066) {
                    skipCheckRD = true;
                } else {  // if ft_diffVar > 14.9066
                    if (ft_blockVar <= 41.1042) {
                        skipCheckRD = false;
                    } else {  // if ft_blockVar > 41.1042
                        skipCheckRD = false;
                    }
                }
            } else {  // if ft_qp > 24.5000
                if (ft_qp <= 29.5000) {
                    if (ft_blockVar <= 66.9946) {
                        skipCheckRD = true;
                    } else {  // if ft_blockVar > 66.9946
                        if (ft_diffVar <= 10.0899) {
                            skipCheckRD = true;
                        } else {  // if ft_diffVar > 10.0899
                            skipCheckRD = false;
                        }
                    }
                } else {  // if ft_qp > 29.5000
                    skipCheckRD = true;
                }
            }
        }
    } else {  // if ft_blockVar > 96.8052
        if (ft_qp <= 29.5000) {
            if (ft_height <= 1620.0000) {
                if (ft_diffVar <= 6.9687) {
                    if (ft_diffVar <= 4.6312) {
                        skipCheckRD = true;
                    } else {  // if ft_diffVar > 4.6312
                        if (ft_qp <= 24.5000) {
                            skipCheckRD = false;
                        } else {  // if ft_qp > 24.5000
                            skipCheckRD = true;
                        }
                    }
                } else {  // if ft_diffVar > 6.9687
                    if (ft_qp <= 24.5000) {
                        if (ft_blockVar <= 181.0269) {
                            skipCheckRD = false;
                        } else {  // if ft_blockVar > 181.0269
                            if (ft_height <= 780.0000) {
                                skipCheckRD = false;
                            } else {  // if ft_height > 780.0000
                                if (ft_config <= 0.5000) {
                                    if (ft_diffVar <= 26.8353) {
                                        skipCheckRD = false;
                                    } else {  // if ft_diffVar > 26.8353
                                        skipCheckRD = false;
                                    }
                                } else {  // if ft_config > 0.5000
                                    skipCheckRD = false;
                                }
                            }
                        }
                    } else {  // if ft_qp > 24.5000
                        if (ft_diffVar <= 16.1158) {
                            skipCheckRD = false;
                        } else {  // if ft_diffVar > 16.1158
                            if (ft_blockVar <= 275.7328) {
                                skipCheckRD = false;
                            } else {  // if ft_blockVar > 275.7328
                                if (ft_diffVar <= 40.3612) {
                                    skipCheckRD = false;
                                } else {  // if ft_diffVar > 40.3612
                                    if (ft_height <= 780.0000) {
                                        skipCheckRD = false;
                                    } else {  // if ft_height > 780.0000
                                        skipCheckRD = false;
                                    }
                                }
                            }
                        }
                    }
                }
            } else {  // if ft_height > 1620.0000
                if (ft_diffVar <= 42.8131) {
                    if (ft_diffVar <= 8.9567) {
                        if (ft_diffVar <= 7.3388) {
                            skipCheckRD = true;
                        } else {  // if ft_diffVar > 7.3388
                            skipCheckRD = true;
                        }
                    } else {  // if ft_diffVar > 8.9567
                        if (ft_qp <= 24.5000) {
                            if (ft_config <= 0.5000) {
                                if (ft_diffVar <= 22.4023) {
                                    skipCheckRD = true;
                                } else {  // if ft_diffVar > 22.4023
                                    skipCheckRD = false;
                                }
                            } else {  // if ft_config > 0.5000
                                if (ft_diffVar <= 23.4266) {
                                    if (ft_blockVar <= 229.7059) {
                                        skipCheckRD = false;
                                    } else {  // if ft_blockVar > 229.7059
                                        skipCheckRD = false;
                                    }
                                } else {  // if ft_diffVar > 23.4266
                                    skipCheckRD = false;
                                }
                            }
                        } else {  // if ft_qp > 24.5000
                            if (ft_diffVar <= 20.1688) {
                                skipCheckRD = true;
                            } else {  // if ft_diffVar > 20.1688
                                skipCheckRD = false;
                            }
                        }
                    }
                } else {  // if ft_diffVar > 42.8131
                    if (ft_qp <= 24.5000) {
                        if (ft_blockVar <= 295.1980) {
                            if (ft_diffVar <= 111.3450) {
                                skipCheckRD = false;
                            } else {  // if ft_diffVar > 111.3450
                                if (ft_diffVar <= 349.4206) {
                                    if (ft_blockVar <= 122.0824) {
                                        skipCheckRD = false;
                                    } else {  // if ft_blockVar > 122.0824
                                        skipCheckRD = false;
                                    }
                                } else {  // if ft_diffVar > 349.4206
                                    skipCheckRD = false;
                                }
                            }
                        } else {  // if ft_blockVar > 295.1980
                            if (ft_diffVar <= 76.3851) {
                                if (ft_config <= 0.5000) {
                                    skipCheckRD = false;
                                } else {  // if ft_config > 0.5000
                                    skipCheckRD = false;
                                }
                            } else {  // if ft_diffVar > 76.3851
                                skipCheckRD = false;
                            }
                        }
                    } else {  // if ft_qp > 24.5000
                        if (ft_blockVar <= 294.6317) {
                            if (ft_blockVar <= 180.3718) {
                                skipCheckRD = false;
                            } else {  // if ft_blockVar > 180.3718
                                if (ft_diffVar <= 100.8230) {
                                    skipCheckRD = false;
                                } else {  // if ft_diffVar > 100.8230
                                    skipCheckRD = false;
                                }
                            }
                        } else {  // if ft_blockVar > 294.6317
                            if (ft_diffVar <= 86.6676) {
                                skipCheckRD = false;
                            } else {  // if ft_diffVar > 86.6676
                                skipCheckRD = false;
                            }
                        }
                    }
                }
            }
        } else {  // if ft_qp > 29.5000
            if (ft_height <= 1620.0000) {
                if (ft_diffVar <= 45.0465) {
                    if (ft_diffVar <= 15.7289) {
                        skipCheckRD = true;
                    } else {  // if ft_diffVar > 15.7289
                        if (ft_qp <= 34.5000) {
                            skipCheckRD = false;
                        } else {  // if ft_qp > 34.5000
                            skipCheckRD = true;
                        }
                    }
                } else {  // if ft_diffVar > 45.0465
                    if (ft_qp <= 34.5000) {
                        if (ft_blockVar <= 217.1561) {
                            skipCheckRD = false;
                        } else {  // if ft_blockVar > 217.1561
                            if (ft_height <= 780.0000) {
                                skipCheckRD = false;
                            } else {  // if ft_height > 780.0000
                                if (ft_blockVar <= 1092.5097) {
                                    skipCheckRD = false;
                                } else {  // if ft_blockVar > 1092.5097
                                    skipCheckRD = false;
                                }
                            }
                        }
                    } else {  // if ft_qp > 34.5000
                        if (ft_blockVar <= 497.7296) {
                            if (ft_blockVar <= 159.6653) {
                                skipCheckRD = true;
                            } else {  // if ft_blockVar > 159.6653
                                if (ft_diffVar <= 83.1295) {
                                    skipCheckRD = true;
                                } else {  // if ft_diffVar > 83.1295
                                    skipCheckRD = false;
                                }
                            }
                        } else {  // if ft_blockVar > 497.7296
                            if (ft_diffVar <= 122.0768) {
                                skipCheckRD = false;
                            } else {  // if ft_diffVar > 122.0768
                                skipCheckRD = false;
                            }
                        }
                    }
                }
            } else {  // if ft_height > 1620.0000
                if (ft_diffVar <= 107.4025) {
                    if (ft_diffVar <= 42.1961) {
                        skipCheckRD = true;
                    } else {  // if ft_diffVar > 42.1961
                        skipCheckRD = true;
                    }
                } else {  // if ft_diffVar > 107.4025
                    if (ft_qp <= 34.5000) {
                        if (ft_blockVar <= 294.6317) {
                            if (ft_diffVar <= 587.0747) {
                                skipCheckRD = false;
                            } else {  // if ft_diffVar > 587.0747
                                skipCheckRD = true;
                            }
                        } else {  // if ft_blockVar > 294.6317
                            skipCheckRD = false;
                        }
                    } else {  // if ft_qp > 34.5000
                        if (ft_blockVar <= 268.3882) {
                            skipCheckRD = true;
                        } else {  // if ft_blockVar > 268.3882
                            skipCheckRD = false;
                        }
                    }
                }
            }
        }
    }
}
            break;
        case 3:
            if (ft_blockVar <= 123.0955) {
                if (ft_blockVar <= 44.7808) {
                    if (ft_blockVar <= 26.4710) {
                        if (ft_blockVar <= 15.4877) {
                            skipCheckRD = true;
                        } else {  // if ft_blockVar > 15.4877
                            if (ft_qp <= 24.5000) {
                                skipCheckRD = true;
                            } else {  // if ft_qp > 24.5000
                                skipCheckRD = true;
                            }
                        }
                    } else {  // if ft_blockVar > 26.4710
                        if (ft_qp <= 24.5000) {
                            if (ft_diffVar <= 10.7010) {
                                skipCheckRD = true;
                            } else {  // if ft_diffVar > 10.7010
                                skipCheckRD = true;
                            }
                        } else {  // if ft_qp > 24.5000
                            skipCheckRD = true;
                        }
                    }
                } else {  // if ft_blockVar > 44.7808
                    if (ft_qp <= 24.5000) {
                        if (ft_previousSplit <= 0.5000) {
                            if (ft_diffVar <= 8.8114) {
                                if (ft_diffVar <= 5.8383) {
                                    skipCheckRD = true;
                                } else {  // if ft_diffVar > 5.8383
                                    skipCheckRD = true;
                                }
                            } else {  // if ft_diffVar > 8.8114
                                if (ft_diffVar <= 42.2163) {
                                    if (ft_blockVar <= 81.3671) {
                                        skipCheckRD = true;
                                    } else {  // if ft_blockVar > 81.3671
                                        skipCheckRD = true;
                                    }
                                } else {  // if ft_diffVar > 42.2163
                                    if (ft_height <= 780.0000) {
                                        skipCheckRD = false;
                                    } else {  // if ft_height > 780.0000
                                        if (ft_blockVar <= 56.4727) {
                                            skipCheckRD = true;
                                        } else {  // if ft_blockVar > 56.4727
                                            skipCheckRD = true;
                                        }
                                    }
                                }
                            }
                        } else {  // if ft_previousSplit > 0.5000
                            if (ft_diffVar <= 38.0817) {
                                skipCheckRD = false;
                            } else {  // if ft_diffVar > 38.0817
                                skipCheckRD = false;
                            }
                        }
                    } else {  // if ft_qp > 24.5000
                        if (ft_qp <= 29.5000) {
                            if (ft_diffVar <= 25.3571) {
                                skipCheckRD = true;
                            } else {  // if ft_diffVar > 25.3571
                                if (ft_previousSplit <= 0.5000) {
                                    if (ft_blockVar <= 81.1921) {
                                        skipCheckRD = true;
                                    } else {  // if ft_blockVar > 81.1921
                                        if (ft_height <= 780.0000) {
                                            skipCheckRD = false;
                                        } else {  // if ft_height > 780.0000
                                            if (ft_config <= 0.5000) {
                                                skipCheckRD = true;
                                            } else {  // if ft_config > 0.5000
                                                if (ft_height <= 1620.0000) {
                                                    skipCheckRD = true;
                                                } else {  // if ft_height > 1620.0000
                                                    skipCheckRD = true;
                                                }
                                            }
                                        }
                                    }
                                } else {  // if ft_previousSplit > 0.5000
                                    skipCheckRD = false;
                                }
                            }
                        } else {  // if ft_qp > 29.5000
                            skipCheckRD = true;
                        }
                    }
                }
            } else {  // if ft_blockVar > 123.0955
    if (ft_previousSplit <= 0.5000) {
        if (ft_height <= 1620.0000) {
            if (ft_diffVar <= 17.1998) {
                if (ft_qp <= 24.5000) {
                    if (ft_diffVar <= 6.7777) {
                        if (ft_diffVar <= 4.7688) {
                            skipCheckRD = true;
                        } else {  // if ft_diffVar > 4.7688
                            skipCheckRD = true;
                        }
                    } else {  // if ft_diffVar > 6.7777
                        if (ft_config <= 0.5000) {
                            if (ft_blockVar <= 1307.1943) {
                                skipCheckRD = true;
                            } else {  // if ft_blockVar > 1307.1943
                                skipCheckRD = false;
                            }
                        } else {  // if ft_config > 0.5000
                            skipCheckRD = false;
                        }
                    }
                } else {  // if ft_qp > 24.5000
                    if (ft_diffVar <= 10.7484) {
                        skipCheckRD = true;
                    } else {  // if ft_diffVar > 10.7484
                        if (ft_qp <= 29.5000) {
                            skipCheckRD = true;
                        } else {  // if ft_qp > 29.5000
                            skipCheckRD = true;
                        }
                    }
                }
            } else {  // if ft_diffVar > 17.1998
                if (ft_qp <= 29.5000) {
                    if (ft_blockVar <= 342.5451) {
                        if (ft_height <= 780.0000) {
                            skipCheckRD = false;
                        } else {  // if ft_height > 780.0000
                            if (ft_qp <= 24.5000) {
                                if (ft_blockVar <= 258.2138) {
                                    skipCheckRD = false;
                                } else {  // if ft_blockVar > 258.2138
                                    skipCheckRD = false;
                                }
                            } else {  // if ft_qp > 24.5000
                                skipCheckRD = false;
                            }
                        }
                    } else {  // if ft_blockVar > 342.5451
                        if (ft_height <= 780.0000) {
                            if (ft_diffVar <= 47.0171) {
                                if (ft_qp <= 24.5000) {
                                    skipCheckRD = false;
                                } else {  // if ft_qp > 24.5000
                                    skipCheckRD = true;
                                }
                            } else {  // if ft_diffVar > 47.0171
                                skipCheckRD = false;
                            }
                        } else {  // if ft_height > 780.0000
                            if (ft_diffVar <= 144.3188) {
                                if (ft_config <= 0.5000) {
                                    skipCheckRD = false;
                                } else {  // if ft_config > 0.5000
                                    skipCheckRD = false;
                                }
                            } else {  // if ft_diffVar > 144.3188
                                if (ft_blockVar <= 2437.4552) {
                                    if (ft_qp <= 24.5000) {
                                        skipCheckRD = false;
                                    } else {  // if ft_qp > 24.5000
                                        skipCheckRD = false;
                                    }
                                } else {  // if ft_blockVar > 2437.4552
                                    skipCheckRD = false;
                                }
                            }
                        }
                    }
                } else {  // if ft_qp > 29.5000
                    if (ft_diffVar <= 216.1429) {
                        if (ft_qp <= 34.5000) {
                            if (ft_diffVar <= 56.7029) {
                                skipCheckRD = true;
                            } else {  // if ft_diffVar > 56.7029
                                if (ft_blockVar <= 192.3672) {
                                    skipCheckRD = true;
                                } else {  // if ft_blockVar > 192.3672
                                    skipCheckRD = true;
                                }
                            }
                        } else {  // if ft_qp > 34.5000
                            skipCheckRD = true;
                        }
                    } else {  // if ft_diffVar > 216.1429
                        if (ft_qp <= 34.5000) {
                            if (ft_blockVar <= 815.8410) {
                                if (ft_blockVar <= 312.7824) {
                                    skipCheckRD = true;
                                } else {  // if ft_blockVar > 312.7824
                                    skipCheckRD = false;
                                }
                            } else {  // if ft_blockVar > 815.8410
                                if (ft_blockVar <= 2921.6858) {
                                    skipCheckRD = false;
                                } else {  // if ft_blockVar > 2921.6858
                                    skipCheckRD = false;
                                }
                            }
                        } else {  // if ft_qp > 34.5000
                            if (ft_blockVar <= 481.8282) {
                                skipCheckRD = true;
                            } else {  // if ft_blockVar > 481.8282
                                if (ft_blockVar <= 1763.8785) {
                                    skipCheckRD = true;
                                } else {  // if ft_blockVar > 1763.8785
                                    skipCheckRD = false;
                                }
                            }
                        }
                    }
                }
            }
        } else {  // if ft_height > 1620.0000
            if (ft_qp <= 29.5000) {
                if (ft_diffVar <= 83.1115) {
                    if (ft_diffVar <= 12.4128) {
                        skipCheckRD = true;
                    } else {  // if ft_diffVar > 12.4128
                        if (ft_qp <= 24.5000) {
                            if (ft_config <= 0.5000) {
                                if (ft_diffVar <= 29.9218) {
                                    skipCheckRD = true;
                                } else {  // if ft_diffVar > 29.9218
                                    skipCheckRD = true;
                                }
                            } else {  // if ft_config > 0.5000
                                skipCheckRD = true;
                            }
                        } else {  // if ft_qp > 24.5000
                            if (ft_config <= 0.5000) {
                                skipCheckRD = true;
                            } else {  // if ft_config > 0.5000
                                skipCheckRD = true;
                            }
                        }
                    }
                } else {  // if ft_diffVar > 83.1115
                    if (ft_qp <= 24.5000) {
                        if (ft_blockVar <= 243.7045) {
                            skipCheckRD = false;
                        } else {  // if ft_blockVar > 243.7045
                            if (ft_diffVar <= 249.5880) {
                                skipCheckRD = false;
                            } else {  // if ft_diffVar > 249.5880
                                skipCheckRD = false;
                            }
                        }
                    } else {  // if ft_qp > 24.5000
                        if (ft_diffVar <= 220.5815) {
                            skipCheckRD = true;
                        } else {  // if ft_diffVar > 220.5815
                            if (ft_blockVar <= 286.0244) {
                                skipCheckRD = true;
                            } else {  // if ft_blockVar > 286.0244
                                skipCheckRD = true;
                            }
                        }
                    }
                }
            } else {  // if ft_qp > 29.5000
                if (ft_qp <= 34.5000) {
                    if (ft_diffVar <= 65.6737) {
                        skipCheckRD = true;
                    } else {  // if ft_diffVar > 65.6737
                        if (ft_diffVar <= 256.9353) {
                            skipCheckRD = true;
                        } else {  // if ft_diffVar > 256.9353
                            skipCheckRD = true;
                        }
                    }
                } else {  // if ft_qp > 34.5000
                    if (ft_diffVar <= 175.5595) {
                        skipCheckRD = true;
                    } else {  // if ft_diffVar > 175.5595
                        if (ft_blockVar <= 1508.2782) {
                            skipCheckRD = true;
                        } else {  // if ft_blockVar > 1508.2782
                            skipCheckRD = true;
                        }
                    }
                }
            }
        }
    } else {  // if ft_previousSplit > 0.5000
        if (ft_height <= 1620.0000) {
            if (ft_diffVar <= 86.9858) {
                if (ft_diffVar <= 6.4403) {
                    skipCheckRD = true;
                } else {  // if ft_diffVar > 6.4403
                    if (ft_qp <= 24.5000) {
                        skipCheckRD = false;
                    } else {  // if ft_qp > 24.5000
                        skipCheckRD = false;
                    }
                }
            } else {  // if ft_diffVar > 86.9858
                if (ft_qp <= 29.5000) {
                    if (ft_blockVar <= 419.8432) {
                        skipCheckRD = false;
                    } else {  // if ft_blockVar > 419.8432
                        skipCheckRD = false;
                    }
                } else {  // if ft_qp > 29.5000
                    if (ft_qp <= 34.5000) {
                        skipCheckRD = false;
                    } else {  // if ft_qp > 34.5000
                        skipCheckRD = false;
                    }
                }
            }
        } else {  // if ft_height > 1620.0000
            if (ft_qp <= 24.5000) {
                if (ft_diffVar <= 16.2940) {
                    skipCheckRD = true;
                } else {  // if ft_diffVar > 16.2940
                    skipCheckRD = false;
                }
            } else {  // if ft_qp > 24.5000
                if (ft_qp <= 29.5000) {
                    skipCheckRD = false;
                } else {  // if ft_qp > 29.5000
                    skipCheckRD = true;
                }
            }
        }
    }
}
            break;
        case 4:
            if (ft_blockVar <= 98.7392) {
                if (ft_blockVar <= 44.8656) {
                    skipCheckRD = true;
                } else {  // if ft_blockVar > 44.8656
                    if (ft_qp <= 24.5000) {
                        if (ft_diffVar <= 6.2790) {
                            skipCheckRD = true;
                        } else {  // if ft_diffVar > 6.2790
                            skipCheckRD = true;
                        }
                    } else {  // if ft_qp > 24.5000
                        skipCheckRD = true;
                    }
                }
            } else {  // if ft_blockVar > 98.7392
    if (ft_height <= 1620.0000) {
        if (ft_blockVar <= 533.3225) {
            if (ft_qp <= 29.5000) {
                if (ft_diffVar <= 48.3004) {
                    if (ft_diffVar <= 7.7013) {
                        skipCheckRD = true;
                    } else {  // if ft_diffVar > 7.7013
                        skipCheckRD = true;
                    }
                } else {  // if ft_diffVar > 48.3004
                    if (ft_qp <= 24.5000) {
                        skipCheckRD = false;
                    } else {  // if ft_qp > 24.5000
                        skipCheckRD = false;
                    }
                }
            } else {  // if ft_qp > 29.5000
                skipCheckRD = true;
            }
        } else {  // if ft_blockVar > 533.3225
            if (ft_qp <= 24.5000) {
                if (ft_diffVar <= 8.7018) {
                    skipCheckRD = true;
                } else {  // if ft_diffVar > 8.7018
                    skipCheckRD = false;
                }
            } else {  // if ft_qp > 24.5000
                if (ft_diffVar <= 23.6998) {
                    skipCheckRD = true;
                } else {  // if ft_diffVar > 23.6998
                    skipCheckRD = false;
                }
            }
        }
    } else {  // if ft_height > 1620.0000
        if (ft_qp <= 24.5000) {
            skipCheckRD = true;
        } else {  // if ft_qp > 24.5000
            skipCheckRD = true;
        }
    }
}
    }

#if ENABLE_TIME_PROFILE
  TimeProfiler::stop(DT_MODEL);
#endif
}