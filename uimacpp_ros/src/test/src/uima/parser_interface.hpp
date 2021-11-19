/** \file parser_interface.hpp .

 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.


   \brief  Interface class for parsers.

-------------------------------------------------------------------------- */
#ifndef UIMA_PARSER_INTERFACE_HPP
#define UIMA_PARSER_INTERFACE_HPP


// ---------------------------------------------------------------------------
//  Includes
// ---------------------------------------------------------------------------

#include "uima/pragmas.hpp" //must be first to surpress warnings
#include <iostream>

namespace uima {

  class ParserConfiguration;
  class TextAnalysisEngine;
  class CAS;

  /**
   * The class ParserInterface is used as an abstract base class for all
   * document parsers.
   * This is just a sketch of how this interface should be used.
   * Things like (the facade?) creating instances of objects implementing
   * this interface are not thought through yet.
   * <pre>
   * TAE engine = createTAE...
   * ParserFacade parserFacade(engine);
   * parserFacade.setMultiDocCallback(...); // optional for mulit-doc formats
   * // for each supported parser beyond the pre-defined ones
   * parserFacade.registerParserForType(p, t, config);
   * for each document {
   *    parserFacade.parseDocument(d,[config]);    // pre-fills the CAS
   *    engine.process(...);  // annotators fill the CAS
   *    ... read out results ...
   *    engine.reset(...)     // flush CAS
   * }
   * engine.destroy();
   * </pre>
   */
  class ParserInterface {
  public:
    /**
     * Callback interface to make it possible for applications to get
     * notified every time an embedded document is done, so  that
     * they can retrieve their results
     *
     * @see ParserInterface::setMultiDocCallback
     */
    class MultiDocCallbackInterface {
    public:
      /**
       * Called <em>after</em> a multi-doc parser detects the end
       * of document.
       *
       * An application should call Engine::processDocument() there
       * and retrieve the results of document processing after that
       * using iterators over the CAS or TCAS.
       * Finally an application should call resetDocument().
       *
       * An application should <em>not</em> call addDocPartsFinish()
       * or addDocPartsFinish() in this function since a conforming
       * parser is supposed to do any doc part processing.
       */
      virtual
      void documentBoundaryReached(UChar const * cpBuffer, size_t uiLength) = 0;
    };

    /**
     * Initialize the parser.
     * The parser is beeing passed an engine object and not a CAS because
     * for (XML) files that contain multiple documents the parser must be
     * able to call the process functions for each embedded document.
     *
     * An implementation needs to store the argument objects for later
     * use in function parse().
     *
     * Called once per session.
     *
     * @param config     The configuration for the parser
     * @param engine     The engine object into which the results go
     * @param fallback   An encoding to use in
     *                   case the parser can't determine the encoding
     *                   by other means
     *
     * @return           UIMA_ERR_NONE if OK, error code otherwise
     */
    virtual
    TyErrorId init(ParserConfiguration const & config, TextAnalysisEngine & engine, const char * = "Latin1" ) = 0;
    /**
     * @see MultiDocCallbackInterface
     */
    virtual
    void setMultiDocCallback(MultiDocCallbackInterface & callbackObject) = 0;

    /**
     * Do the parsing add the tag free text to the CAS and potentially
     * translate tag information to CAS annotation.
     *
     * Called once per document.
     *
     * @param inputFileName The input to process
     * @return              UIMA_ERR_NONE if OK, error code otherwise
     */
    virtual
    TyErrorId parseDocument(char* const inputFileName) = 0;
    virtual
    TyErrorId parseDocument(std::istream & inputFileStream) = 0;

    /**
     * Returns the number of documents parsed by the parser.
     * This will always be 1 for HTML but can be more for XML.
     *
     * Optionally called once per document.
     */
    virtual
    size_t getNumberOfDocumentsParsed() const = 0;

    /**
     * Returns the number of bytes parsed by the parser.
     * Information function to allow throughput computation by calling
     * environment.
     *
     * Optionally called once per document.
     */
    virtual
    size_t getNumberOfBytesParsed() const = 0;

    /**
     * De-initialize the parser (free ressources etc.)
     *
     * Called once per session.
     *
     * @return           UIMA_ERR_NONE if OK, error code otherwise
     */
    virtual
    TyErrorId deInit() = 0;
  }
  ; /* ParserInterface */

} // namespace uima

#endif //UIMA_PARSER_INTERFACE_HPP

