// ==========================================================================
// ACME
//
// Copyright (c) D-ICE Engineering.
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
//
// ==========================================================================


#ifndef ACME_VERSION_H
#define ACME_VERSION_H

#include <string>

namespace acme {
  namespace git {

    // Is the metadata populated? We may not have metadata if
    // there wasn't a .git directory (e.g. downloaded source
    // code without revision history).
    bool IsPopulated();

    // Were there any uncommitted changes that won't be reflected
    // in the CommitID?
    bool AnyUncommittedChanges();

    // The commit author's name.
    std::string AuthorName();

    // The commit author's email.
    std::string AuthorEmail();

    // The commit SHA1.
    std::string CommitSHA1();

    // The ISO8601 commit date.
    std::string CommitDate();

    // The last tag past this commit
    std::string LastTag();

    // The current branch where the current commit lies
    std::string CurrentBranch();

    // Get the normalized version string
    std::string GetNormalizedVersionString();

    // The project name as seen by CMake
    std::string ProjectName();

  }

}  // end namespace acme

#endif //ACME_VERSION_H
