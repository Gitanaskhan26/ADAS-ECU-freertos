#!/bin/bash
# Generate code coverage report for ADAS ECU
# Requirements: gcov, lcov, genhtml

set -e

# Get script directory and workspace root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Change to workspace root
cd "$WORKSPACE_ROOT"

echo "========================================="
echo "  ADAS ECU - Coverage Report Generator"
echo "  Workspace: $WORKSPACE_ROOT"
echo "========================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check for required tools
echo -e "\n${YELLOW}[1/7]${NC} Checking dependencies..."
command -v gcov >/dev/null 2>&1 || { echo -e "${RED}Error: gcov not installed${NC}"; exit 1; }
command -v lcov >/dev/null 2>&1 || { echo -e "${RED}Error: lcov not installed. Run: apt install lcov${NC}"; exit 1; }

echo -e "${GREEN}✓${NC} gcov found: $(gcov --version | head -1)"
echo -e "${GREEN}✓${NC} lcov found: $(lcov --version | head -1)"

# Clean previous coverage data
echo -e "\n${YELLOW}[2/7]${NC} Cleaning previous build..."
rm -rf build
echo -e "${GREEN}✓${NC} Cleaned"

# Build with coverage instrumentation
echo -e "\n${YELLOW}[3/7]${NC} Building with coverage flags..."
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Coverage ..
cmake --build . --target adas_ecu -- -j$(nproc)
make -C tests all -j$(nproc)
cd ..
echo -e "${GREEN}✓${NC} Build complete with coverage instrumentation"

# Run all unit tests to generate .gcda files
echo -e "\n${YELLOW}[4/7]${NC} Running unit tests..."

test_count=0
passed_count=0

# Temporarily disable exit on error for test execution
set +e

for test_exe in build/tests/test_*; do
    if [ -x "$test_exe" ] && [ -f "$test_exe" ]; then
        test_name=$(basename "$test_exe")
        echo -n "  Running $test_name... "
        
        timeout 10 $test_exe > /dev/null 2>&1
        exit_code=$?
        
        if [ $exit_code -eq 0 ]; then
            echo -e "${GREEN}PASSED${NC}"
            ((passed_count++))
        elif [ $exit_code -eq 124 ]; then
            echo -e "${RED}TIMEOUT${NC}"
        else
            echo -e "${RED}FAILED (exit $exit_code)${NC}"
        fi
        ((test_count++))
    fi
done

# Re-enable exit on error
set -e

echo -e "${GREEN}✓${NC} Tests completed: $passed_count/$test_count passed"

# Capture coverage data
echo -e "\n${YELLOW}[5/7]${NC} Capturing coverage data..."
mkdir -p build/coverage_html

lcov --capture \
     --directory build \
     --output-file build/coverage.info \
     --rc lcov_branch_coverage=1 \
     2>&1 | grep -v "geninfo: WARNING"

echo -e "${GREEN}✓${NC} Coverage data captured"

# Filter out system headers and test files
echo -e "\n${YELLOW}[6/7]${NC} Filtering coverage data..."
lcov --remove build/coverage.info \
     '/usr/*' \
     '*/FreeRTOS-Kernel/*' \
     '*/tests/*' \
     '*/build/*' \
     --output-file build/coverage_filtered.info \
     --rc lcov_branch_coverage=1 \
     2>&1 | grep -v "geninfo: WARNING"

echo -e "${GREEN}✓${NC} Filtered out system/test files"

# Generate HTML report
echo -e "\n${YELLOW}[7/7]${NC} Generating HTML report..."
genhtml build/coverage_filtered.info \
        --output-directory build/coverage_html \
        --title "ADAS ECU Coverage Report" \
        --legend \
        --branch-coverage \
        --rc genhtml_branch_coverage=1 \
        2>&1 | grep -v "genhtml: WARNING"

echo -e "${GREEN}✓${NC} HTML report generated"

# Display summary
echo -e "\n========================================="
echo -e "  ${GREEN}Coverage Report Summary${NC}"
echo -e "========================================="

lcov --summary build/coverage_filtered.info --rc lcov_branch_coverage=1 2>&1 | \
    grep -E "lines\.\.\.|branches\.\.\." | \
    sed 's/^/  /'

echo ""
echo -e "📊 ${GREEN}Full report:${NC} build/coverage_html/index.html"
echo -e "📄 ${GREEN}Raw data:${NC} build/coverage_filtered.info"
echo ""

# Check if we meet ASIL-D target (90% MC/DC)
branch_coverage=$(lcov --summary build/coverage_filtered.info --rc lcov_branch_coverage=1 2>&1 | \
    grep "branches" | sed 's/.*: //' | sed 's/%.*//')

line_coverage=$(lcov --summary build/coverage_filtered.info --rc lcov_branch_coverage=1 2>&1 | \
    grep "lines" | sed 's/.*: //' | sed 's/%.*//')

echo "Branch Coverage: ${branch_coverage}%"
echo "Line Coverage: ${line_coverage}%"

if (( $(echo "$branch_coverage >= 90" | bc -l) )); then
    echo -e "${GREEN}✓ ASIL-D Target Met:${NC} Branch coverage ≥ 90%"
else
    echo -e "${YELLOW}⚠ ASIL-D Target:${NC} Branch coverage ${branch_coverage}% (target: 90%)"
fi

if (( $(echo "$line_coverage >= 90" | bc -l) )); then
    echo -e "${GREEN}✓ ASIL-D Target Met:${NC} Line coverage ≥ 90%"
else
    echo -e "${YELLOW}⚠ ASIL-D Target:${NC} Line coverage ${line_coverage}% (target: 90%)"
fi

echo ""
echo "========================================="
echo -e "  ${GREEN}Coverage Generation Complete${NC}"
echo "========================================="
echo ""
echo "To view the report:"
echo "  firefox build/coverage_html/index.html"
echo "  # or"
echo "  open build/coverage_html/index.html"
echo ""
