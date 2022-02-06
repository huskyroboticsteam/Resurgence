#pragma once

//
//  base64 encoding and decoding with C++.
//  Version: 2.rc.08 (release candidate)
//  Taken from:
//  https://renenyffenegger.ch/notes/development/Base64/Encoding-and-decoding-base-64-with-cpp/
//

#include <string>

#if __cplusplus >= 201703L
#include <string_view>
#endif // __cplusplus >= 201703L

namespace base64 {

/**
 * @brief Encode the given data as base64.
 *
 * @param s The data to encode.
 * @param url Set to true if the data being encoded is a url.
 * @return std::string The base64 encoded string.
 */
std::string base64_encode(std::string const& s, bool url = false);

/**
 * @brief Encode the given data in base64 PEM (Privacy-enhanced Electronic Mail) format.
 * This adds newlines periodically. For communication purposes, clients
 * most likely want base64::base64_encode().
 *
 * @param s The data to encode.
 * @return std::string The base64 encoded string.
 */
std::string base64_encode_pem(std::string const& s);

/**
 * @brief Encode the given data in base64 MIME format. For communication purposes, clients
 * most likely want base64::base64_encode().
 *
 * @param s The data to encode.
 * @return std::string The base64 encoded string.
 */
std::string base64_encode_mime(std::string const& s);

/**
 * @brief Decode the given string from base64.
 *
 * @param s The base64 encoded string to decode.
 * @param remove_linebreaks If true, remove linebreaks from the base64 string.
 * @return std::string The decoded data.
 */
std::string base64_decode(std::string const& s, bool remove_linebreaks = false);

/**
 * @brief Encode the given data as base64.
 *
 * @param len The length of the data array.
 * @param url Set to true if the data being encoded is a url.
 * @return std::string
 */
std::string base64_encode(unsigned char const*, size_t len, bool url = false);

#if __cplusplus >= 201703L
//
// Interface with std::string_view rather than const std::string&
// Requires C++17
// Provided by Yannic Bonenberger (https://github.com/Yannic)
//

/**
 * @brief Encode the given data as base64.
 *
 * @param s The data to encode.
 * @param url Set to true if the data being encoded is a url.
 * @return std::string The base64 encoded string.
 */
std::string base64_encode(std::string_view s, bool url = false);

/**
 * @brief Encode the given data in base64 PEM (Privacy-enhanced Electronic Mail) format.
 * This adds newlines periodically. For communication purposes, clients
 * most likely want base64::base64_encode().
 *
 * @param s The data to encode.
 * @return std::string The base64 encoded string.
 */
std::string base64_encode_pem(std::string_view s);

/**
 * @brief Encode the given data in base64 MIME format. For communication purposes, clients
 * most likely want base64::base64_encode().
 *
 * @param s The data to encode.
 * @return std::string The base64 encoded string.
 */
std::string base64_encode_mime(std::string_view s);

/**
 * @brief Decode the given string from base64.
 *
 * @param s The base64 encoded string to decode.
 * @param remove_linebreaks If true, remove linebreaks from the base64 string.
 * @return std::string The decoded data.
 */
std::string base64_decode(std::string_view s, bool remove_linebreaks = false);
#endif // __cplusplus >= 201703L

} // namespace base64
