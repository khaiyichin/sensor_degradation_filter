#!/usr/bin/env bash

# Default values
EPS=false

# Function to display usage information
usage() {
    echo "Usage: $0 [-e] <DRAWIO_FILE>"
    echo "Options:"
    echo "  -e   Create an EPS copy for the PDF files (optional)"
    exit 1
}

# Parse options
while getopts "e" opt
do
    case "${opt}" in
        e) EPS=true ;;
        \?) usage ;;
    esac
done
shift $((OPTIND - 1))

# Check if the number of arguments passed is correct
if [ "$#" -ne 1 ]
then
	usage
fi

# Assign the first argument to DRAWIO_FILE
DRAWIO_FILE="$1"

# Validate the extension
if [[ ! "$DRAWIO_FILE" =~ \.drawio$ ]]
then
    echo "Error: File must have .drawio extension."
    exit 1
fi

# Extract the page names into an array
DIAGRAM_PAGES=($(grep -oP '(?<=name=")[^"]+' ${DRAWIO_FILE}))

echo ${DIAGRAM_PAGES[*]}

for (( p = 0; p < ${#DIAGRAM_PAGES[@]}; p++ ))
do
	# Ignore pages that have the `NO_EXPORT` prefix
	if [[ "${DIAGRAM_PAGES[p]}" != NO_EXPORT* ]]
	then

		# Define output filename
		OUTNAME="${DIAGRAM_PAGES[p]}.pdf"
		echo "Exporting ${OUTNAME}..."

		drawio --export \
			--format pdf \
			--crop \
			--page-index ${p} \
			--output ${OUTNAME} \
			${DRAWIO_FILE}

		echo "Done!"
		# echo "Adding white background to ${OUTNAME}..."

		# sed -i "s|<defs/>|<defs/><rect fill=\"#ffffff\" width=\"100%\" height=\"100%\" x=\"0\" y=\"0\"/>|g" ${OUTNAME}

		# echo "Done!"

		# Create an EPS copy
		if ${EPS}
		then
			echo "Creating ${DIAGRAM_PAGES[p]}.eps..."
			inkscape ${DIAGRAM_PAGES[p]}.pdf -o ${DIAGRAM_PAGES[p]}.eps --export-ignore-filters --export-ps-level=3 --export-text-to-path
			echo "Done!"
		fi

		echo -e "\n"
	fi
done
