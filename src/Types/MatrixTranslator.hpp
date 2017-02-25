/*
 * MatrixTranslator.hpp
 *
 *  Created on: Dec 15, 2012
 *      Author: maciej
 */

#ifndef MATRIXTRANSLATOR_HPP_
#define MATRIXTRANSLATOR_HPP_

#include <opencv2/core/core.hpp>

#include <vector>
#include <string>

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

namespace Types {

class MatrixTranslator {
public:
    static cv::Mat fromStr(const std::string & s) {

        typedef std::vector< std::string > split_vector_type;


        split_vector_type rows; // #2: Search for tokens
        boost::split( rows, s, boost::is_any_of(";"), boost::token_compress_on );

        std::vector<split_vector_type> mat;
        mat.resize(rows.size());
        for (int i = 0; i < rows.size(); ++i) {
        	boost::trim( rows[i] );
        	boost::split( mat[i], rows[i], boost::is_any_of(" ,"), boost::token_compress_on );
        }

        int r = rows.size();
        int c = mat[0].size();

        cv::Mat ret(r, c, CV_32FC1);
        for (int rr = 0; rr < r; ++rr) {
        	for (int cc = 0; cc < c; ++cc) {
        		ret.at<float>(rr, cc) = boost::lexical_cast<float>(mat[rr][cc]);
                        std::cout << ret.at<float>(rr, cc) << " ";
        	}
        }

        return ret;
    }

    static std::string toStr(cv::Mat m) {
    	std::stringstream ss;
    	std::string delim = "";
        for(int r = 0; r < m.rows; ++r) {
        	ss << delim;
        	for(int c = 0; c < m.cols; ++c) {
                        switch(m.type()) {
                                case CV_8UC1: ss << m.at<uint8_t>(r, c) << " "; break;
                                case CV_8SC1: ss << m.at<int8_t>(r, c) << " "; break;
                                case CV_16UC1: ss << m.at<uint16_t>(r, c) << " "; break;
                                case CV_16SC1: ss << m.at<int16_t>(r, c) << " "; break;
                                case CV_32SC1: ss << m.at<int32_t>(r, c) << " "; break;
                                case CV_32FC1: ss << m.at<float>(r, c) << " "; break;
                                case CV_64FC1: ss << m.at<double>(r, c) << " "; break;
                                default: ss << "0 ";
                        }
        	}
        	delim = "; ";
        }
        return ss.str();
    }

    static cv::Mat fromStr(const std::string & s, const int DATA_FORMAT) {
        std::vector<double> values;

        typedef std::vector< std::string > split_vector_type;

        split_vector_type rows; // #2: Search for tokens
        boost::split( rows, s, boost::is_any_of(";"), boost::token_compress_on );

        std::vector<split_vector_type> mat;
        mat.resize(rows.size());
        for (int i = 0; i < rows.size(); ++i) {
        	boost::trim( rows[i] );
        	boost::split( mat[i], rows[i], boost::is_any_of(" ,"), boost::token_compress_on );
        }

        int r = rows.size();
        int c = mat[0].size();

        cv::Mat ret(r, c, DATA_FORMAT);
        for (int rr = 0; rr < r; ++rr) {
        	for (int cc = 0; cc < c; ++cc) {
        		ret.at<double>(rr, cc) = boost::lexical_cast<double>(mat[rr][cc]);
        	}
        }

        return ret;
    }
};

}

#endif /* MATRIXTRANSLATOR_HPP_ */
